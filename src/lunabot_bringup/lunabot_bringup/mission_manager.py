"""Shuttle-cycle mission manager FSM for autonomous rover operation."""

from __future__ import annotations

import math
from enum import IntEnum

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Float32, Int32, String

from lunabot_bringup.mission_timer import MissionTimer
from lunabot_interfaces.action import Deposit, Excavate

_INHIBIT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

_OPERATOR_STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

MISSION_LIMIT_S = 1200.0


class MissionState(IntEnum):
    """
    States for the mission manager FSM.

    Each constant names a single phase in the shuttle-cycle mission sequence.
    """

    INITIALIZE_MISSION = 0
    ACQUIRE_TAG = 1
    TURN_TO_EXCAVATION = 2
    NAV_TO_EXCAVATION = 3
    EXCAVATE = 4
    TURN_TO_DEPOSITION = 5
    NAV_TO_DEPOSITION = 6
    DEPOSIT = 7
    CHECK_NEXT_CYCLE_TIME = 8
    SAFE_FAIL = 9
    PREHOC_TRAVERSAL = 11
    HALT = 99
    HALT_MISSION = HALT


class MissionManager(Node):
    """
    ROS 2 node implementing the shuttle-cycle mission FSM.

    Drives the rover through repeated excavation and deposition cycles,
    using a MissionTimer to gate whether another cycle can begin within
    the 1200-second mission budget.
    """

    def __init__(self) -> None:
        """Initialise the node, declare parameters, and create action clients."""
        super().__init__("mission_manager")

        self.declare_parameter("prehoc_v_estimated_mps", 0.3)
        self.declare_parameter("prehoc_t_margin_s", 60.0)
        self.declare_parameter("prehoc_s_start_exc_m", 5.0)
        self.declare_parameter("prehoc_s_exc_dep_m", 5.0)

        self.declare_parameter("waypoint_mid_obstacle_x", 3.0)
        self.declare_parameter("waypoint_mid_obstacle_y", -1.1)
        self.declare_parameter("waypoint_mid_obstacle_yaw", 0.0)
        self.declare_parameter("waypoint_mid_obstacle_enabled", True)
        self.declare_parameter("waypoint_excavation_x", 6.0)
        self.declare_parameter("waypoint_excavation_y", -1.5)
        self.declare_parameter("waypoint_excavation_yaw", 0.0)
        self.declare_parameter("waypoint_deposition_x", 0.3)
        self.declare_parameter("waypoint_deposition_y", -2.8)
        self.declare_parameter("waypoint_deposition_yaw", 0.0)
        self.declare_parameter("nav_goal_timeout_s", 120.0)

        self.declare_parameter("max_nav_retries", 2)
        self.declare_parameter("max_excavation_retries", 1)
        self.declare_parameter("max_deposition_retries", 1)
        self.declare_parameter("max_shuttle_cycles", 10)

        self._max_nav_retries = self.get_parameter(
            "max_nav_retries"
        ).value
        self._max_exc_retries = self.get_parameter(
            "max_excavation_retries"
        ).value
        self._max_dep_retries = self.get_parameter(
            "max_deposition_retries"
        ).value
        self._max_cycles = self.get_parameter(
            "max_shuttle_cycles"
        ).value
        self._cycle_count = 0
        self._estop_active = False
        self._motion_inhibited = False
        self._last_failure_reason = ""
        self._mission_active = False

        self._state: MissionState = MissionState.INITIALIZE_MISSION
        self._timer: MissionTimer | None = None

        self._navigate_client: ActionClient = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose_gate",
        )
        self._excavate_client: ActionClient = ActionClient(
            self,
            Excavate,
            "/mission/excavate",
        )
        self._deposit_client: ActionClient = ActionClient(
            self,
            Deposit,
            "/mission/deposit",
        )

        self._inhibit_sub = self.create_subscription(
            Bool,
            "/safety/motion_inhibit",
            self._inhibit_cb,
            _INHIBIT_QOS,
        )
        self._estop_sub = self.create_subscription(
            Bool, "/safety/estop", self._estop_cb, 10
        )
        self._mission_state_pub = self.create_publisher(
            String, "/mission/state", _OPERATOR_STATE_QOS
        )
        self._autonomy_mode_pub = self.create_publisher(
            String, "/mission/autonomy_mode", _OPERATOR_STATE_QOS
        )
        self._time_remaining_pub = self.create_publisher(
            Float32, "/mission/time_remaining_s", _OPERATOR_STATE_QOS
        )
        self._cycle_count_pub = self.create_publisher(
            Int32, "/mission/cycle_count", _OPERATOR_STATE_QOS
        )
        self._failure_reason_pub = self.create_publisher(
            String, "/mission/last_failure_reason", _OPERATOR_STATE_QOS
        )
        self._operator_state_timer = self.create_timer(
            1.0, self._publish_operator_state
        )
        self._publish_operator_state()

    def _inhibit_cb(self, msg: Bool) -> None:
        """Track motion inhibit for safety gating."""
        self._motion_inhibited = msg.data

    def _estop_cb(self, msg: Bool) -> None:
        """Track E-stop state."""
        self._estop_active = msg.data

    def _is_safe(self) -> bool:
        """Return True if the rover is safe to operate."""
        return not self._estop_active and not self._motion_inhibited

    def _state_text(self) -> str:
        """Return the current mission state as lower-case operator text."""
        return self._state.name.lower()

    def _autonomy_mode_text(self) -> str:
        """Return a passive operator-facing autonomy mode."""
        if self._estop_active:
            return "estop"
        if self._motion_inhibited:
            return "motion_inhibited"
        if self._state == MissionState.HALT_MISSION:
            return "halted"
        if self._state == MissionState.SAFE_FAIL:
            return "safe_fail"
        if self._mission_active:
            return "autonomous"
        return "idle"

    def _time_remaining_s(self) -> float:
        """Return remaining competition mission time in seconds."""
        if self._timer is None:
            return MISSION_LIMIT_S
        try:
            elapsed = float(self._timer.elapsedMissionTime())
        except (AttributeError, TypeError, ValueError):
            return MISSION_LIMIT_S
        if not math.isfinite(elapsed):
            return MISSION_LIMIT_S
        elapsed = max(0.0, elapsed)
        return max(0.0, MISSION_LIMIT_S - elapsed)

    def _publish_operator_state(self) -> None:
        """Publish passive mission telemetry for Foxglove and bags."""
        state_msg = String()
        state_msg.data = self._state_text()
        self._mission_state_pub.publish(state_msg)

        mode_msg = String()
        mode_msg.data = self._autonomy_mode_text()
        self._autonomy_mode_pub.publish(mode_msg)

        time_msg = Float32()
        time_msg.data = float(self._time_remaining_s())
        self._time_remaining_pub.publish(time_msg)

        cycle_msg = Int32()
        cycle_msg.data = int(self._cycle_count)
        self._cycle_count_pub.publish(cycle_msg)

        failure_msg = String()
        failure_msg.data = self._last_failure_reason
        self._failure_reason_pub.publish(failure_msg)

    def _set_failure_reason(self, reason: str) -> None:
        """Record the latest mission failure reason and publish it."""
        self._last_failure_reason = reason
        self._publish_operator_state()

    def _retry_action(
        self,
        action_fn,
        label: str,
        max_retries: int,
    ) -> tuple[bool, str]:
        """Call action_fn up to max_retries+1 times."""
        for attempt in range(max_retries + 1):
            if not self._is_safe():
                return False, f"{label}: safety stop active"
            success, detail = action_fn()
            if success:
                return True, detail
            self.get_logger().warn(
                f"{label} attempt {attempt + 1}/"
                f"{max_retries + 1} failed: {detail}"
            )
        return False, f"{label}: exhausted {max_retries + 1} attempts"

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def run_mission(self) -> None:
        """Advance the FSM until HALT_MISSION is reached."""
        self._mission_active = True
        self._publish_operator_state()
        while self._state != MissionState.HALT_MISSION:
            if not self._is_safe():
                reason = "Safety stop during mission"
                self.get_logger().error(f"{reason} — halting")
                self._set_failure_reason(reason)
                self._state = MissionState.HALT_MISSION
                self._publish_operator_state()
                break
            self._step()

        self._mission_active = False
        self._publish_operator_state()
        self.get_logger().info(
            f"Mission halted after {self._cycle_count} cycles."
        )

    # ------------------------------------------------------------------
    # FSM dispatcher
    # ------------------------------------------------------------------

    def _step(self) -> None:
        """Dispatch the current state to its handler and advance."""
        handler = {
            MissionState.INITIALIZE_MISSION: self._handle_initialize_mission,
            MissionState.ACQUIRE_TAG: self._handle_acquire_tag,
            MissionState.PREHOC_TRAVERSAL: self._handle_prehoc_traversal,
            MissionState.TURN_TO_EXCAVATION: self._handle_turn_to_excavation,
            MissionState.NAV_TO_EXCAVATION: self._handle_nav_to_excavation,
            MissionState.EXCAVATE: self._handle_excavate,
            MissionState.TURN_TO_DEPOSITION: self._handle_turn_to_deposition,
            MissionState.NAV_TO_DEPOSITION: self._handle_nav_to_deposition,
            MissionState.DEPOSIT: self._handle_deposit,
            MissionState.CHECK_NEXT_CYCLE_TIME: self._handle_check_next_cycle_time,
            MissionState.SAFE_FAIL: self._handle_safe_fail,
        }.get(self._state)

        if handler is None:
            self.get_logger().error(
                f"No handler for state {self._state!r}; halting."
            )
            self._state = MissionState.HALT_MISSION
            return

        next_state = handler()
        if next_state != self._state:
            self._state = next_state
            self._publish_operator_state()

    # ------------------------------------------------------------------
    # State handlers
    # ------------------------------------------------------------------

    def _handle_initialize_mission(self) -> MissionState:
        """Initialise mission state and create the cycle timer."""
        self.get_logger().info("Initialising mission.")
        self._timer = MissionTimer()
        return MissionState.ACQUIRE_TAG

    def _handle_acquire_tag(self) -> MissionState:
        """
        Begin a new cycle and acquire the localisation tag.

        On the first cycle (``completedCycles == 0``) transitions to
        ``PREHOC_TRAVERSAL`` to gate entry using a conservative time estimate.
        On later cycles, transitions directly to ``TURN_TO_EXCAVATION``.
        """
        self.get_logger().info("Acquiring localisation tag.")
        if self._timer is not None:
            self._timer.beginCycle()

        if self._timer is not None and self._timer.completedCycles == 0:
            return MissionState.PREHOC_TRAVERSAL

        return MissionState.TURN_TO_EXCAVATION

    def _safe_float_parameter(self, name: str) -> float | None:
        """Convert a ROS parameter value to float, returning None on failure."""
        raw_value = self.get_parameter(name).value

        if raw_value is None:
            self.get_logger().error(f"Parameter '{name}' is unset.")
            return None

        if isinstance(raw_value, bool) or not isinstance(
            raw_value, int | float | str | bytes
        ):
            self.get_logger().error(
                f"Parameter '{name}' has unsupported type "
                f"{type(raw_value).__name__}."
            )
            return None

        try:
            return float(raw_value)
        except (TypeError, ValueError):
            self.get_logger().error(
                f"Parameter '{name}' with value {raw_value!r} is not numeric."
            )
            return None

    def _handle_prehoc_traversal(self) -> MissionState:
        """
        Gate the first cycle using a conservative pre-hoc traversal estimate.

        Computes ``T_prehoc = (S_start_exc / v) + (S_exc_dep / v) + T_margin``
        and asks the timer whether the mission budget allows proceeding.
        Transitions to ``TURN_TO_EXCAVATION`` if allowed, ``HALT_MISSION``
        otherwise.
        """
        v = self._safe_float_parameter("prehoc_v_estimated_mps")
        t_margin = self._safe_float_parameter("prehoc_t_margin_s")
        s_start_exc = self._safe_float_parameter("prehoc_s_start_exc_m")
        s_exc_dep = self._safe_float_parameter("prehoc_s_exc_dep_m")

        if (
            v is None
            or t_margin is None
            or s_start_exc is None
            or s_exc_dep is None
        ):
            self._set_failure_reason("Invalid pre-hoc traversal parameters")
            return MissionState.HALT_MISSION

        if not math.isfinite(v) or v <= 0.0:
            reason = (
                "Invalid pre-hoc traversal parameter "
                "'prehoc_v_estimated_mps'; expected a finite value > 0.0."
            )
            self.get_logger().error(reason)
            self._set_failure_reason(reason)
            return MissionState.HALT_MISSION

        invalid_non_negative_params = []
        if not math.isfinite(t_margin) or t_margin < 0.0:
            invalid_non_negative_params.append("prehoc_t_margin_s")
        if not math.isfinite(s_start_exc) or s_start_exc < 0.0:
            invalid_non_negative_params.append("prehoc_s_start_exc_m")
        if not math.isfinite(s_exc_dep) or s_exc_dep < 0.0:
            invalid_non_negative_params.append("prehoc_s_exc_dep_m")

        if invalid_non_negative_params:
            invalid_param_names = ", ".join(invalid_non_negative_params)
            reason = (
                "Invalid pre-hoc traversal parameter(s) "
                f"{invalid_param_names}; expected finite values >= 0.0."
            )
            self.get_logger().error(reason)
            self._set_failure_reason(reason)
            return MissionState.HALT_MISSION

        t_prehoc = (s_start_exc / v) + (s_exc_dep / v) + t_margin

        if self._timer is not None and self._timer.canStartCycle(
            pre_hoc_time_s=t_prehoc
        ):
            self.get_logger().info(
                f"Pre-hoc estimate {t_prehoc:.1f}s fits budget; proceeding."
            )
            return MissionState.TURN_TO_EXCAVATION

        self.get_logger().info(
            f"Pre-hoc estimate {t_prehoc:.1f}s would exceed budget; halting."
        )
        self._set_failure_reason(
            f"Pre-hoc estimate {t_prehoc:.1f}s would exceed mission budget"
        )
        return MissionState.HALT_MISSION

    def _handle_turn_to_excavation(self) -> MissionState:
        """Turn the rover to face the excavation zone."""
        self.get_logger().info("Turning to excavation zone.")
        return MissionState.NAV_TO_EXCAVATION

    def _handle_nav_to_excavation(self) -> MissionState:
        """Navigate to the excavation zone with bounded retries."""
        self.get_logger().info("Navigating to excavation zone.")
        success, detail = self._retry_action(
            self._send_navigate_to_excavation,
            "nav_to_excavation",
            self._max_nav_retries,
        )
        if not success:
            reason = f"Navigation to excavation failed: {detail}"
            self.get_logger().error(reason)
            self._set_failure_reason(reason)
            return MissionState.SAFE_FAIL
        return MissionState.EXCAVATE

    def _handle_excavate(self) -> MissionState:
        """Execute the excavation action with bounded retries."""
        self.get_logger().info("Starting excavation.")
        success, detail = self._retry_action(
            self._send_excavate,
            "excavation",
            self._max_exc_retries,
        )
        if not success:
            reason = f"Excavation failed: {detail}"
            self.get_logger().error(reason)
            self._set_failure_reason(reason)
            return MissionState.SAFE_FAIL
        return MissionState.TURN_TO_DEPOSITION

    def _handle_turn_to_deposition(self) -> MissionState:
        """Turn the rover to face the deposition zone."""
        self.get_logger().info("Turning to deposition zone.")
        return MissionState.NAV_TO_DEPOSITION

    def _handle_nav_to_deposition(self) -> MissionState:
        """Navigate to the deposition zone with bounded retries."""
        self.get_logger().info("Navigating to deposition zone.")
        success, detail = self._retry_action(
            self._send_navigate_to_deposition,
            "nav_to_deposition",
            self._max_nav_retries,
        )
        if not success:
            reason = f"Navigation to deposition failed: {detail}"
            self.get_logger().error(reason)
            self._set_failure_reason(reason)
            return MissionState.SAFE_FAIL
        return MissionState.DEPOSIT

    def _handle_deposit(self) -> MissionState:
        """Execute the deposit action with bounded retries."""
        self.get_logger().info("Starting deposit.")
        success, detail = self._retry_action(
            self._send_deposit,
            "deposition",
            self._max_dep_retries,
        )
        if not success:
            reason = f"Deposit failed: {detail}"
            self.get_logger().error(reason)
            self._set_failure_reason(reason)
            return MissionState.SAFE_FAIL
        return MissionState.CHECK_NEXT_CYCLE_TIME

    def _handle_check_next_cycle_time(self) -> MissionState:
        """Record cycle time and decide whether to start another cycle."""
        if self._timer is not None:
            self._timer.recordCycleTime()

        self._cycle_count += 1
        self.get_logger().info(
            f"Completed cycle {self._cycle_count}/"
            f"{self._max_cycles}"
        )

        if self._cycle_count >= self._max_cycles:
            self.get_logger().info(
                "Max shuttle cycles reached; halting."
            )
            return MissionState.HALT_MISSION

        if self._timer is not None and self._timer.canStartCycle():
            self.get_logger().info(
                "Time budget allows another cycle."
            )
            return MissionState.ACQUIRE_TAG

        self.get_logger().info(
            "Time budget exhausted; halting mission."
        )
        return MissionState.HALT_MISSION

    def _handle_safe_fail(self) -> MissionState:
        """Log a safe-fail condition and transition to halt."""
        self.get_logger().error("Safe-fail triggered; halting mission.")
        return MissionState.HALT_MISSION

    # ------------------------------------------------------------------
    # Action helpers (injectable for testing)
    # ------------------------------------------------------------------

    def _wait_for_future(self, future, timeout_s: float, timeout_detail: str):
        """Spin until a future completes and return its result."""
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if future.done():
            return future.result()
        return timeout_detail

    def _wait_for_server(
        self,
        client: ActionClient,
        action_name: str,
        timeout_s: float = 30.0,
    ) -> tuple[bool, str]:
        """Block until an action server is available."""
        if client.wait_for_server(timeout_sec=timeout_s):
            return True, f"{action_name} server available"
        return False, f"{action_name} server unavailable after {timeout_s}s"

    def _build_nav_goal(self, x: float, y: float, yaw: float) -> NavigateToPose.Goal:
        """Build a NavigateToPose goal in the map frame."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def _mid_obstacle_enabled(self) -> bool:
        """Return whether mission travel should route through the midpoint."""
        value = self.get_parameter("waypoint_mid_obstacle_enabled").value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    def _waypoint_goal(self, prefix: str) -> NavigateToPose.Goal | None:
        """Build one configured waypoint goal, returning None on bad params."""
        x = self._safe_float_parameter(f"waypoint_{prefix}_x")
        y = self._safe_float_parameter(f"waypoint_{prefix}_y")
        yaw = self._safe_float_parameter(f"waypoint_{prefix}_yaw")
        if x is None or y is None or yaw is None:
            return None
        return self._build_nav_goal(x, y, yaw)

    def _send_nav_goal(self, goal, label: str) -> tuple[bool, str]:
        """Send a NavigateToPose goal and wait for the result."""
        available, detail = self._wait_for_server(
            self._navigate_client, "/navigate_to_pose_gate"
        )
        if not available:
            return False, detail

        timeout = self._safe_float_parameter("nav_goal_timeout_s")
        if timeout is None:
            return False, f"{label}: nav_goal_timeout_s parameter invalid"

        self.get_logger().info(
            f"{label}: goal at ({goal.pose.pose.position.x:.2f}, "
            f"{goal.pose.pose.position.y:.2f})"
        )
        send_future = self._navigate_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(
            send_future, 30.0, f"{label} goal response timed out"
        )
        return self._check_goal_result(goal_handle, label, timeout)

    def _send_nav_sequence(
        self,
        goals: list[tuple[str, NavigateToPose.Goal]],
        label: str,
    ) -> tuple[bool, str]:
        """Send configured navigation goals in order."""
        completed: list[str] = []
        for goal_label, goal in goals:
            if not self._is_safe():
                return False, f"{label}: safety stop active before {goal_label}"
            success, detail = self._send_nav_goal(goal, goal_label)
            if not success:
                return False, detail
            completed.append(goal_label)

        return True, f"{label} succeeded via {', '.join(completed)}"

    def _send_navigate_to_excavation(self) -> tuple[bool, str]:
        """Send a NavigateToPose goal toward the excavation zone."""
        excavation_goal = self._waypoint_goal("excavation")
        if excavation_goal is None:
            return False, "excavation waypoint parameters invalid"

        goals = []
        if self._mid_obstacle_enabled():
            midpoint_goal = self._waypoint_goal("mid_obstacle")
            if midpoint_goal is None:
                return False, "mid-obstacle waypoint parameters invalid"
            goals.append(("navigate_to_mid_obstacle", midpoint_goal))
        goals.append(("navigate_to_excavation", excavation_goal))

        return self._send_nav_sequence(goals, "navigate_to_excavation")

    def _send_navigate_to_deposition(self) -> tuple[bool, str]:
        """Send a NavigateToPose goal toward the deposition zone."""
        deposition_goal = self._waypoint_goal("deposition")
        if deposition_goal is None:
            return False, "deposition waypoint parameters invalid"

        goals = []
        if self._mid_obstacle_enabled():
            midpoint_goal = self._waypoint_goal("mid_obstacle")
            if midpoint_goal is None:
                return False, "mid-obstacle waypoint parameters invalid"
            goals.append(("navigate_to_mid_obstacle", midpoint_goal))
        goals.append(("navigate_to_deposition", deposition_goal))

        return self._send_nav_sequence(goals, "navigate_to_deposition")

    def _send_excavate(self) -> tuple[bool, str]:
        """Send an Excavate action goal."""
        available, detail = self._wait_for_server(
            self._excavate_client, "/mission/excavate"
        )
        if not available:
            return False, detail

        goal = Excavate.Goal()
        goal.mode = Excavate.Goal.MODE_AUTO
        goal.timeout_s = 60.0

        send_future = self._excavate_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(
            send_future, 30.0, "excavate goal response timed out"
        )
        return self._check_goal_result(goal_handle, "excavate", goal.timeout_s + 5.0)

    def _send_deposit(self) -> tuple[bool, str]:
        """Send a Deposit action goal."""
        available, detail = self._wait_for_server(
            self._deposit_client, "/mission/deposit"
        )
        if not available:
            return False, detail

        goal = Deposit.Goal()
        goal.mode = Deposit.Goal.MODE_AUTO
        goal.timeout_s = 30.0

        send_future = self._deposit_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(
            send_future, 30.0, "deposit goal response timed out"
        )
        return self._check_goal_result(goal_handle, "deposit", goal.timeout_s + 5.0)

    def _check_goal_result(
        self,
        goal_handle,
        action_name: str,
        result_timeout_s: float,
    ) -> tuple[bool, str]:
        """Check the outcome of a goal handle and return success/detail."""
        if isinstance(goal_handle, str):
            return False, goal_handle
        if goal_handle is None or not goal_handle.accepted:
            return False, f"{action_name} goal rejected"

        result_future = goal_handle.get_result_async()
        result = self._wait_for_future(
            result_future, result_timeout_s, f"{action_name} result timed out"
        )
        if isinstance(result, str):
            return False, result
        if result is None:
            return False, f"{action_name} result unavailable"
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            return False, f"{action_name} finished with status {result.status}"

        return True, f"{action_name} succeeded"


def main(args=None) -> None:
    """Entry point for the mission_manager executable."""
    rclpy.init(args=args)
    node = MissionManager()
    try:
        node.run_mission()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
