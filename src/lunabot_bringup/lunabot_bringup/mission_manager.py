"""Shuttle-cycle mission manager FSM for autonomous rover operation."""

from __future__ import annotations

import math
from enum import IntEnum

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from lunabot_bringup.mission_timer import MissionTimer
from lunabot_interfaces.action import Deposit, Excavate


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

        self.declare_parameter("use_sim_time", False)
        self.declare_parameter("prehoc_v_estimated_mps", 0.3)
        self.declare_parameter("prehoc_t_margin_s", 60.0)
        self.declare_parameter("prehoc_s_start_exc_m", 5.0)
        self.declare_parameter("prehoc_s_exc_dep_m", 5.0)

        self.declare_parameter("waypoint_mid_obstacle_x", 3.0)
        self.declare_parameter("waypoint_mid_obstacle_y", -1.1)
        self.declare_parameter("waypoint_mid_obstacle_yaw", 0.0)
        self.declare_parameter("waypoint_excavation_x", 6.0)
        self.declare_parameter("waypoint_excavation_y", -1.5)
        self.declare_parameter("waypoint_excavation_yaw", 0.0)
        self.declare_parameter("waypoint_deposition_x", 0.3)
        self.declare_parameter("waypoint_deposition_y", -2.8)
        self.declare_parameter("waypoint_deposition_yaw", -1.5708)
        self.declare_parameter("nav_goal_timeout_s", 120.0)

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

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def run_mission(self) -> None:
        """Advance the FSM until HALT_MISSION is reached."""
        while self._state != MissionState.HALT_MISSION:
            self._step()

        self.get_logger().info("Mission halted.")

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

        self._state = handler()

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
            return MissionState.HALT_MISSION

        if not math.isfinite(v) or v <= 0.0:
            self.get_logger().error(
                "Invalid pre-hoc traversal parameter "
                "'prehoc_v_estimated_mps'; expected a finite value > 0.0."
            )
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
            self.get_logger().error(
                "Invalid pre-hoc traversal parameter(s) "
                f"{invalid_param_names}; expected finite values >= 0.0."
            )
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
        return MissionState.HALT_MISSION

    def _handle_turn_to_excavation(self) -> MissionState:
        """Turn the rover to face the excavation zone."""
        self.get_logger().info("Turning to excavation zone.")
        return MissionState.NAV_TO_EXCAVATION

    def _handle_nav_to_excavation(self) -> MissionState:
        """Navigate to the excavation zone."""
        self.get_logger().info("Navigating to excavation zone.")
        success, detail = self._send_navigate_to_excavation()
        if not success:
            self.get_logger().error(f"Navigation to excavation failed: {detail}")
            return MissionState.SAFE_FAIL
        return MissionState.EXCAVATE

    def _handle_excavate(self) -> MissionState:
        """Execute the excavation action."""
        self.get_logger().info("Starting excavation.")
        success, detail = self._send_excavate()
        if not success:
            self.get_logger().error(f"Excavation failed: {detail}")
            return MissionState.SAFE_FAIL
        return MissionState.TURN_TO_DEPOSITION

    def _handle_turn_to_deposition(self) -> MissionState:
        """Turn the rover to face the deposition zone."""
        self.get_logger().info("Turning to deposition zone.")
        return MissionState.NAV_TO_DEPOSITION

    def _handle_nav_to_deposition(self) -> MissionState:
        """Navigate to the deposition zone."""
        self.get_logger().info("Navigating to deposition zone.")
        success, detail = self._send_navigate_to_deposition()
        if not success:
            self.get_logger().error(f"Navigation to deposition failed: {detail}")
            return MissionState.SAFE_FAIL
        return MissionState.DEPOSIT

    def _handle_deposit(self) -> MissionState:
        """Execute the deposit action."""
        self.get_logger().info("Starting deposit.")
        success, detail = self._send_deposit()
        if not success:
            self.get_logger().error(f"Deposit failed: {detail}")
            return MissionState.SAFE_FAIL
        return MissionState.CHECK_NEXT_CYCLE_TIME

    def _handle_check_next_cycle_time(self) -> MissionState:
        """Record cycle time and decide whether to start another cycle."""
        if self._timer is not None:
            self._timer.recordCycleTime()

        if self._timer is not None and self._timer.canStartCycle():
            self.get_logger().info("Time budget allows another cycle.")
            return MissionState.ACQUIRE_TAG

        self.get_logger().info("Time budget exhausted; halting mission.")
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

    def _send_nav_goal(self, goal, label: str) -> tuple[bool, str]:
        """Send a NavigateToPose goal and wait for the result."""
        available, detail = self._wait_for_server(
            self._navigate_client, "/navigate_to_pose"
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

    def _send_navigate_to_excavation(self) -> tuple[bool, str]:
        """Send a NavigateToPose goal toward the excavation zone."""
        x = self._safe_float_parameter("waypoint_excavation_x")
        y = self._safe_float_parameter("waypoint_excavation_y")
        yaw = self._safe_float_parameter("waypoint_excavation_yaw")
        if x is None or y is None or yaw is None:
            return False, "excavation waypoint parameters invalid"

        goal = self._build_nav_goal(x, y, yaw)
        return self._send_nav_goal(goal, "navigate_to_excavation")

    def _send_navigate_to_deposition(self) -> tuple[bool, str]:
        """Send a NavigateToPose goal toward the deposition zone."""
        x = self._safe_float_parameter("waypoint_deposition_x")
        y = self._safe_float_parameter("waypoint_deposition_y")
        yaw = self._safe_float_parameter("waypoint_deposition_yaw")
        if x is None or y is None or yaw is None:
            return False, "deposition waypoint parameters invalid"

        goal = self._build_nav_goal(x, y, yaw)
        return self._send_nav_goal(goal, "navigate_to_deposition")

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
