"""Excavation action adapter."""

from contextlib import suppress
from math import isfinite
from time import monotonic, sleep

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from lunabot_interfaces.action import Excavate
from lunabot_interfaces.msg import ExcavationStatus, ExcavationTelemetry

STATE_TO_PHASE = {
    ExcavationStatus.STATE_IDLE: Excavate.Feedback.PHASE_PRECHECK,
    ExcavationStatus.STATE_HOMING: Excavate.Feedback.PHASE_PRECHECK,
    ExcavationStatus.STATE_READY: Excavate.Feedback.PHASE_PRECHECK,
    ExcavationStatus.STATE_STARTING: Excavate.Feedback.PHASE_SPINUP,
    ExcavationStatus.STATE_EXCAVATING: Excavate.Feedback.PHASE_DIGGING,
    ExcavationStatus.STATE_STOPPING: Excavate.Feedback.PHASE_RETRACT,
    ExcavationStatus.STATE_FAULT: Excavate.Feedback.PHASE_RETRACT,
}

ACTIVE_STATES = {
    ExcavationStatus.STATE_HOMING,
    ExcavationStatus.STATE_STARTING,
    ExcavationStatus.STATE_EXCAVATING,
    ExcavationStatus.STATE_STOPPING,
}


class ExcavationActionServer(Node):
    """Bridge the mission-facing excavation action onto the controller."""

    def __init__(self):
        """Initialise action server, status subscriptions, and service clients."""
        super().__init__("excavation_action_server")

        self.declare_parameter("nominal_duration_s", 8.0)
        self.declare_parameter("loop_period_s", 0.2)
        self.declare_parameter("stop_timeout_s", 2.0)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._status = None
        self._latest_telemetry = None
        self._callback_group = ReentrantCallbackGroup()

        self.create_subscription(
            ExcavationStatus,
            "/excavation/status",
            self._handle_status,
            qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            ExcavationTelemetry,
            "/excavation/telemetry",
            self._handle_telemetry,
            qos,
            callback_group=self._callback_group,
        )

        self._home_client = self.create_client(
            Trigger,
            "/excavation/home",
            callback_group=self._callback_group,
        )
        self._start_client = self.create_client(
            Trigger,
            "/excavation/start",
            callback_group=self._callback_group,
        )
        self._stop_client = self.create_client(
            Trigger,
            "/excavation/stop",
            callback_group=self._callback_group,
        )

        self._server = ActionServer(
            self,
            Excavate,
            "/mission/excavate",
            execute_callback=self.execute_excavate,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )

    def destroy_node(self):
        """Destroy the action server before node teardown."""
        self._server.destroy()
        super().destroy_node()

    def _handle_status(self, msg: ExcavationStatus):
        """Store the latest excavation controller status."""
        self._status = msg

    def _handle_telemetry(self, msg: ExcavationTelemetry):
        """Store the latest excavation telemetry sample."""
        self._latest_telemetry = msg

    @staticmethod
    def _is_valid_excavation_goal(goal_request):
        """Return True when the goal satisfies the excavation action contract."""
        mode = goal_request.mode
        timeout_s = float(goal_request.timeout_s)
        target_fill_fraction = float(goal_request.target_fill_fraction)
        max_drive_speed_mps = float(goal_request.max_drive_speed_mps)

        if mode not in (
            Excavate.Goal.MODE_AUTO,
            Excavate.Goal.MODE_TELEOP_ASSIST,
        ):
            return False
        if not isfinite(timeout_s):
            return False
        if not isfinite(target_fill_fraction):
            return False
        if not 0.0 <= target_fill_fraction <= 1.0:
            return False
        if not isfinite(max_drive_speed_mps):
            return False
        return not max_drive_speed_mps < 0.0

    def goal_callback(self, goal_request):
        """Accept only excavation goals that match the supported contract."""
        if self._is_valid_excavation_goal(goal_request):
            return GoalResponse.ACCEPT
        return GoalResponse.REJECT

    def cancel_callback(self, _goal_handle):
        """Accept cancellation so the mechanism can be stopped safely."""
        return CancelResponse.ACCEPT

    @property
    def loop_period_s(self):
        """Return the action feedback loop period."""
        return float(self.get_parameter("loop_period_s").value)

    def _result(self, success: bool, reason_code: int, reason: str, duration_s: float):
        """Build an excavation result."""
        result = Excavate.Result()
        result.success = bool(success)
        result.reason_code = int(reason_code)
        result.failure_reason = str(reason)
        # The action contract uses 0.0 to mean unknown or not measured.
        result.collected_mass_kg_estimate = 0.0
        result.duration_s = float(duration_s)
        return result

    def _call_trigger(self, client, timeout_s: float):
        """Call a Trigger service synchronously and return the response."""
        if not client.wait_for_service(timeout_sec=timeout_s):
            return None
        future = client.call_async(Trigger.Request())
        deadline = monotonic() + timeout_s
        while rclpy.ok() and monotonic() < deadline:
            if future.done():
                return future.result()
            sleep(min(self.loop_period_s, 0.05))
        return future.result() if future.done() else None

    def _publish_feedback(self, goal_handle, elapsed: float):
        """Publish action feedback from controller status and telemetry."""
        telemetry = self._latest_telemetry
        state = (
            self._status.state
            if self._status is not None
            else ExcavationStatus.STATE_IDLE
        )

        feedback = Excavate.Feedback()
        feedback.phase = STATE_TO_PHASE.get(state, Excavate.Feedback.PHASE_PRECHECK)
        feedback.elapsed_s = float(elapsed)
        # The action contract uses 0.0 to mean unknown or not measured.
        feedback.fill_fraction_estimate = 0.0
        feedback.excavation_motor_current_a = (
            float(telemetry.motor_current_a)
            if telemetry is not None
            else (
                float(self._status.motor_current_a) if self._status is not None else 0.0
            )
        )
        feedback.jam_detected = (
            self._status is not None
            and self._status.fault_code == ExcavationStatus.FAULT_OVERCURRENT
        )
        feedback.estop_active = (
            self._status.estop_active if self._status is not None else False
        )
        goal_handle.publish_feedback(feedback)

    def _fault_reason(self):
        """Map the latest status fault into an action result code and reason."""
        if self._status is None:
            return Excavate.Result.REASON_SHUTDOWN, "Excavation status unavailable"
        if self._status.fault_code == ExcavationStatus.FAULT_ESTOP:
            return Excavate.Result.REASON_ESTOP, "Excavation estop active"
        if self._status.fault_code == ExcavationStatus.FAULT_OVERCURRENT:
            return (
                Excavate.Result.REASON_JAM_OR_OVERCURRENT,
                "Excavation jam or overcurrent detected",
            )
        if self._status.fault_code == ExcavationStatus.FAULT_DRIVER:
            return Excavate.Result.REASON_DRIVER_FAULT, "Excavation driver fault active"
        return Excavate.Result.REASON_INTERLOCK_BLOCKED, "Excavation controller faulted"

    def _finish_goal(
        self,
        goal_handle,
        *,
        terminal_state: str,
        success: bool,
        reason_code: int,
        reason: str,
        duration_s: float,
    ):
        """Apply a terminal goal state, log it once, and build the result."""
        if terminal_state == "succeed":
            with suppress(RuntimeError):
                goal_handle.succeed()
            self.get_logger().info("Excavation action succeeded")
            return self._result(True, reason_code, reason, duration_s)

        if terminal_state == "cancel":
            with suppress(RuntimeError):
                goal_handle.canceled()
            self.get_logger().info(reason)
            return self._result(success, reason_code, reason, duration_s)

        with suppress(RuntimeError):
            goal_handle.abort()
        self.get_logger().warn(reason)
        return self._result(success, reason_code, reason, duration_s)

    def _current_state(self) -> int:
        """Return the current controller state or idle when no status is available."""
        if self._status is None:
            return ExcavationStatus.STATE_IDLE
        return self._status.state

    def _abort_with_fault(self, goal_handle, duration_s: float):
        """Abort the action using the latest controller fault information."""
        code, reason = self._fault_reason()
        return self._finish_goal(
            goal_handle,
            terminal_state="abort",
            success=False,
            reason_code=code,
            reason=reason,
            duration_s=duration_s,
        )

    def _validate_initial_state(self, goal_handle):
        """Reject excavation when the controller is already faulted or busy."""
        state = self._current_state()
        if state == ExcavationStatus.STATE_FAULT:
            return self._abort_with_fault(goal_handle, duration_s=0.0)

        if state in (ExcavationStatus.STATE_IDLE, ExcavationStatus.STATE_READY):
            return None

        return self._finish_goal(
            goal_handle,
            terminal_state="abort",
            success=False,
            reason_code=Excavate.Result.REASON_INTERLOCK_BLOCKED,
            reason=f"Excavation controller busy in state '{state}'",
            duration_s=0.0,
        )

    def _home_until_ready(
        self,
        goal_handle,
        *,
        timeout_s: float,
        stop_timeout_s: float,
        start_time: float,
    ):
        """Home the controller when needed and resolve any terminal homing outcome."""
        if self._current_state() == ExcavationStatus.STATE_READY:
            return None

        response = self._call_trigger(self._home_client, 2.0)
        if response is None or not response.success:
            return self._finish_goal(
                goal_handle,
                terminal_state="abort",
                success=False,
                reason_code=Excavate.Result.REASON_INTERLOCK_BLOCKED,
                reason="Excavation home command was rejected",
                duration_s=0.0,
            )

        homing_result, elapsed = self._wait_for_ready_or_fault(
            timeout_s,
            goal_handle,
            start_time,
        )
        if homing_result == "ready":
            return None
        if homing_result == "cancel":
            return self._resolve_stop_settle(
                goal_handle,
                elapsed=elapsed,
                stop_timeout_s=stop_timeout_s,
                settled_reason_code=Excavate.Result.REASON_CANCELED,
                settled_reason="Excavation goal canceled during homing",
                settle_timeout_reason=(
                    "Excavation did not settle after cancel during homing"
                ),
                shutdown_reason="Node shutdown during cancel during homing",
            )
        if homing_result == "timeout":
            return self._resolve_stop_settle(
                goal_handle,
                elapsed=elapsed,
                stop_timeout_s=stop_timeout_s,
                settled_reason_code=Excavate.Result.REASON_TIMEOUT,
                settled_reason="Excavation timeout reached during homing",
                settle_timeout_reason="Excavation did not settle after homing timeout",
                shutdown_reason="Node shutdown during homing timeout handling",
            )
        if homing_result == "fault":
            return self._abort_with_fault(goal_handle, duration_s=elapsed)
        return self._finish_goal(
            goal_handle,
            terminal_state="abort",
            success=False,
            reason_code=Excavate.Result.REASON_SHUTDOWN,
            reason="Node shutdown during excavation homing",
            duration_s=elapsed,
        )

    def _start_excavation(self, goal_handle, start_time: float):
        """Send the start command and reject the goal if the controller refuses."""
        start_response = self._call_trigger(self._start_client, 2.0)
        if start_response is not None and start_response.success:
            return None
        return self._finish_goal(
            goal_handle,
            terminal_state="abort",
            success=False,
            reason_code=Excavate.Result.REASON_INTERLOCK_BLOCKED,
            reason="Excavation start command was rejected",
            duration_s=monotonic() - start_time,
        )

    def _resolve_active_outcome(
        self,
        goal_handle,
        *,
        elapsed: float,
        timeout_s: float,
        stop_timeout_s: float,
    ):
        """Resolve cancel, timeout, or fault during an active excavation attempt."""
        if goal_handle.is_cancel_requested:
            return self._resolve_stop_settle(
                goal_handle,
                elapsed=elapsed,
                stop_timeout_s=stop_timeout_s,
                settled_reason_code=Excavate.Result.REASON_CANCELED,
                settled_reason="Excavation goal canceled by client",
                settle_timeout_reason="Excavation did not settle after cancel",
                shutdown_reason="Node shutdown during cancel handling",
            )
        if timeout_s > 0.0 and elapsed >= timeout_s:
            return self._resolve_stop_settle(
                goal_handle,
                elapsed=elapsed,
                stop_timeout_s=stop_timeout_s,
                settled_reason_code=Excavate.Result.REASON_TIMEOUT,
                settled_reason="Excavation timeout reached",
                settle_timeout_reason="Excavation did not settle after timeout",
                shutdown_reason="Node shutdown during timeout handling",
            )
        if self._current_state() == ExcavationStatus.STATE_FAULT:
            return self._abort_with_fault(goal_handle, duration_s=elapsed)
        return None

    def _excavation_finished_successfully(self, saw_excavating: bool):
        """Return True when excavation has entered and then left the active phase."""
        if not saw_excavating:
            return False

        state = self._current_state()
        if state in (
            ExcavationStatus.STATE_READY,
            ExcavationStatus.STATE_IDLE,
        ):
            return True
        return self._is_stopped_without_fault()

    def _run_excavation_loop(
        self,
        goal_handle,
        *,
        start_time: float,
        timeout_s: float,
        nominal_duration: float,
        stop_timeout_s: float,
    ):
        """Drive the bounded excavation loop until success, failure, or shutdown."""
        saw_excavating = False
        stop_requested = False

        while rclpy.ok():
            elapsed = monotonic() - start_time
            outcome = self._resolve_active_outcome(
                goal_handle,
                elapsed=elapsed,
                timeout_s=timeout_s,
                stop_timeout_s=stop_timeout_s,
            )
            if outcome is not None:
                return outcome

            if self._current_state() == ExcavationStatus.STATE_EXCAVATING:
                saw_excavating = True

            if saw_excavating and elapsed >= nominal_duration and not stop_requested:
                self._call_trigger(self._stop_client, stop_timeout_s)
                stop_requested = True

            if self._excavation_finished_successfully(saw_excavating):
                return self._finish_goal(
                    goal_handle,
                    terminal_state="succeed",
                    success=True,
                    reason_code=Excavate.Result.REASON_SUCCESS,
                    reason="",
                    duration_s=elapsed,
                )

            self._publish_feedback(goal_handle, elapsed)
            sleep(self.loop_period_s)

        return self._finish_goal(
            goal_handle,
            terminal_state="abort",
            success=False,
            reason_code=Excavate.Result.REASON_SHUTDOWN,
            reason="Node shutdown during excavation",
            duration_s=monotonic() - start_time,
        )

    def _wait_for_ready_or_fault(
        self, timeout_s: float, goal_handle, start_time: float
    ):
        """Wait until the controller is ready, faulted, canceled, or timed out."""
        while rclpy.ok():
            elapsed = monotonic() - start_time

            if goal_handle.is_cancel_requested:
                return "cancel", elapsed
            if timeout_s > 0.0 and elapsed >= timeout_s:
                return "timeout", elapsed
            if (
                self._status is not None
                and self._status.state == ExcavationStatus.STATE_READY
            ):
                return "ready", elapsed
            if (
                self._status is not None
                and self._status.state == ExcavationStatus.STATE_FAULT
            ):
                return "fault", elapsed

            self._publish_feedback(goal_handle, elapsed)
            sleep(self.loop_period_s)

        return "shutdown", monotonic() - start_time

    def _wait_for_stop_settle(self, timeout_s: float):
        """Wait until the controller settles, faults, times out, or shuts down."""
        settle_start = monotonic()
        while rclpy.ok():
            if (
                self._status is not None
                and self._status.state == ExcavationStatus.STATE_FAULT
            ):
                return "fault"
            if (
                self._latest_telemetry is not None
                and (
                    self._latest_telemetry.estop_active
                    or self._latest_telemetry.driver_fault
                    or self._latest_telemetry.fault_code
                    != ExcavationTelemetry.FAULT_NONE
                )
            ):
                return "fault"
            if self._status is not None and self._status.state not in ACTIVE_STATES:
                return "settled"
            if (
                self._latest_telemetry is not None
                and self._is_stopped_without_fault()
            ):
                return "settled"
            if monotonic() - settle_start >= timeout_s:
                return "timeout"
            sleep(self.loop_period_s)
        return "shutdown"

    def _is_stopped_without_fault(self):
        """Return True when telemetry shows the mechanism has stopped cleanly."""
        telemetry = self._latest_telemetry
        return (
            telemetry is not None
            and not telemetry.estop_active
            and not telemetry.driver_fault
            and telemetry.fault_code == ExcavationTelemetry.FAULT_NONE
            and not telemetry.motor_enabled
        )

    def _resolve_stop_settle(
        self,
        goal_handle,
        *,
        elapsed: float,
        stop_timeout_s: float,
        settled_reason_code: int,
        settled_reason: str,
        settle_timeout_reason: str,
        shutdown_reason: str,
    ):
        """Stop the mechanism and resolve any mixed fault, cancel, or timeout path."""
        self._call_trigger(self._stop_client, stop_timeout_s)
        settle_result = self._wait_for_stop_settle(stop_timeout_s)

        if settle_result == "fault":
            code, reason = self._fault_reason()
            return self._finish_goal(
                goal_handle,
                terminal_state="abort",
                success=False,
                reason_code=code,
                reason=reason,
                duration_s=elapsed,
            )

        if settle_result == "shutdown":
            return self._finish_goal(
                goal_handle,
                terminal_state="abort",
                success=False,
                reason_code=Excavate.Result.REASON_SHUTDOWN,
                reason=shutdown_reason,
                duration_s=elapsed,
            )

        if settle_result == "timeout":
            return self._finish_goal(
                goal_handle,
                terminal_state="abort",
                success=False,
                reason_code=Excavate.Result.REASON_INTERLOCK_BLOCKED,
                reason=settle_timeout_reason,
                duration_s=elapsed,
            )

        terminal_state = (
            "cancel"
            if settled_reason_code == Excavate.Result.REASON_CANCELED
            else "abort"
        )
        return self._finish_goal(
            goal_handle,
            terminal_state=terminal_state,
            success=False,
            reason_code=settled_reason_code,
            reason=settled_reason,
            duration_s=elapsed,
        )

    def execute_excavate(self, goal_handle):
        """Run one bounded excavation attempt through the controller."""
        timeout_s = float(goal_handle.request.timeout_s)
        nominal_duration = float(self.get_parameter("nominal_duration_s").value)
        stop_timeout_s = float(self.get_parameter("stop_timeout_s").value)
        start_time = monotonic()

        invalid_start_state = self._validate_initial_state(goal_handle)
        if invalid_start_state is not None:
            return invalid_start_state

        homing_result = self._home_until_ready(
            goal_handle,
            timeout_s=timeout_s,
            stop_timeout_s=stop_timeout_s,
            start_time=start_time,
        )
        if homing_result is not None:
            return homing_result

        start_result = self._start_excavation(goal_handle, start_time)
        if start_result is not None:
            return start_result

        return self._run_excavation_loop(
            goal_handle,
            start_time=start_time,
            timeout_s=timeout_s,
            nominal_duration=nominal_duration,
            stop_timeout_s=stop_timeout_s,
        )


def main(args=None):
    """Run the excavation action server node."""
    rclpy.init(args=args)
    node = ExcavationActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
