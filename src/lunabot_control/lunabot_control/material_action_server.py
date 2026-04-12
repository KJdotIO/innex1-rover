"""Stub action server for the deposition mission phase."""

from math import isfinite
from time import monotonic, sleep

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from lunabot_interfaces.action import Deposit


class MaterialActionServer(Node):
    """Provide a deposition action stub for mission and integration testing."""

    def __init__(self):
        """Initialise action servers and simulation parameters."""
        super().__init__("material_action_server")

        self.declare_parameter("deposit_nominal_duration_s", 5.0)
        self.declare_parameter("loop_period_s", 0.2)
        self.declare_parameter("force_failure_action", "")

        self._callback_group = ReentrantCallbackGroup()

        self._deposit_server = ActionServer(
            self,
            Deposit,
            "/mission/deposit",
            execute_callback=self.execute_deposit,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info("Deposition action stub ready")

    def destroy_node(self):
        """Destroy action server handles before node teardown."""
        self._deposit_server.destroy()
        super().destroy_node()

    @staticmethod
    def _is_valid_deposit_goal(goal_request):
        """Return True when the goal satisfies the deposit action contract."""
        mode = goal_request.mode
        timeout_s = float(goal_request.timeout_s)
        dump_duration_s = float(goal_request.dump_duration_s)

        if mode not in (
            Deposit.Goal.MODE_AUTO,
            Deposit.Goal.MODE_TELEOP_ASSIST,
        ):
            return False
        if not isfinite(timeout_s):
            return False
        if not isfinite(dump_duration_s):
            return False
        return not dump_duration_s < 0.0

    def goal_callback(self, goal_request):
        """Accept only deposit goals that match the supported contract."""
        if self._is_valid_deposit_goal(goal_request):
            return GoalResponse.ACCEPT
        return GoalResponse.REJECT

    def cancel_callback(self, _goal_handle):
        """Accept each cancel request so clients can stop long-running goals."""
        return CancelResponse.ACCEPT

    @property
    def loop_period_s(self):
        """Return the configured feedback publish period."""
        return float(self.get_parameter("loop_period_s").value)

    def _should_force_failure(self, action_name):
        """Return True when this action should intentionally fail."""
        value = self.get_parameter("force_failure_action").value
        configured = str(value).strip().lower()
        return configured == action_name

    @staticmethod
    def _deposit_result(success, reason_code, reason, residual_fill, duration_s):
        """Build a Deposit result message."""
        result = Deposit.Result()
        result.success = bool(success)
        result.reason_code = int(reason_code)
        result.failure_reason = str(reason)
        # The action contract uses 0.0 to mean unknown or not measured.
        result.residual_fill_fraction_estimate = float(residual_fill)
        result.duration_s = float(duration_s)
        return result

    def _deposit_nominal_duration(self, goal_handle):
        """Return the simulated deposit duration for this goal."""
        nominal_duration = float(self.get_parameter("deposit_nominal_duration_s").value)
        dump_duration = float(goal_handle.request.dump_duration_s)
        if dump_duration > 0.0:
            return max(nominal_duration, dump_duration + 2.0)
        return nominal_duration

    @staticmethod
    def _deposit_phase(progress):
        """Map progress to the current deposit feedback phase."""
        if progress < 0.2:
            return Deposit.Feedback.PHASE_PRECHECK
        if progress < 0.45:
            return Deposit.Feedback.PHASE_OPENING
        if progress < 0.65:
            return Deposit.Feedback.PHASE_RAISING
        if progress < 0.85:
            return Deposit.Feedback.PHASE_DUMPING
        return Deposit.Feedback.PHASE_CLOSING

    def _publish_deposit_feedback(self, goal_handle, elapsed, nominal_duration):
        """Publish one bounded deposit feedback update and return progress."""
        progress = min(elapsed / max(nominal_duration, 0.1), 1.0)

        feedback = Deposit.Feedback()
        feedback.phase = self._deposit_phase(progress)
        feedback.elapsed_s = float(elapsed)
        feedback.actuator_current_a = float(3.5 + 2.5 * progress)
        feedback.door_open = feedback.phase in (
            Deposit.Feedback.PHASE_OPENING,
            Deposit.Feedback.PHASE_RAISING,
            Deposit.Feedback.PHASE_DUMPING,
        )
        feedback.bed_raised = feedback.phase in (
            Deposit.Feedback.PHASE_RAISING,
            Deposit.Feedback.PHASE_DUMPING,
        )
        feedback.estop_active = False
        goal_handle.publish_feedback(feedback)
        return progress

    def _canceled_deposit_result(self, goal_handle, elapsed):
        """Cancel the active goal and return the matching result."""
        goal_handle.canceled()
        return self._deposit_result(
            False,
            Deposit.Result.REASON_CANCELED,
            "Deposit goal canceled by client",
            1.0,
            elapsed,
        )

    def _timeout_deposit_result(self, goal_handle, elapsed):
        """Abort the active goal after a deposit timeout."""
        goal_handle.abort()
        return self._deposit_result(
            False,
            Deposit.Result.REASON_TIMEOUT,
            "Deposition timeout reached",
            1.0,
            elapsed,
        )

    def _forced_failure_result(self, goal_handle, elapsed):
        """Abort the goal when the bench is configured to force failure."""
        goal_handle.abort()
        return self._deposit_result(
            False,
            Deposit.Result.REASON_FORCED_FAILURE,
            "Forced failure for bench testing",
            1.0,
            elapsed,
        )

    def _succeeded_deposit_result(self, goal_handle, elapsed):
        """Mark the goal successful and return the result."""
        goal_handle.succeed()
        return self._deposit_result(
            True,
            Deposit.Result.REASON_SUCCESS,
            "",
            0.05,
            elapsed,
        )

    def _shutdown_deposit_result(self, goal_handle, start):
        """Abort active work during shutdown and report a bounded result."""
        if goal_handle.is_active:
            goal_handle.abort()
        return self._deposit_result(
            False,
            Deposit.Result.REASON_SHUTDOWN,
            "Node shutdown during deposition",
            1.0,
            monotonic() - start,
        )

    def _terminal_deposit_result(self, goal_handle, elapsed, timeout_s):
        """Return a terminal result when the deposit loop should stop."""
        if goal_handle.is_cancel_requested:
            return self._canceled_deposit_result(goal_handle, elapsed)
        if timeout_s > 0.0 and elapsed >= timeout_s:
            return self._timeout_deposit_result(goal_handle, elapsed)
        if self._should_force_failure("deposit"):
            return self._forced_failure_result(goal_handle, elapsed)
        return None

    def execute_deposit(self, goal_handle):
        """Run deposition stub with success, timeout, and failure paths."""
        nominal_duration = self._deposit_nominal_duration(goal_handle)
        timeout_s = float(goal_handle.request.timeout_s)
        start = monotonic()

        while rclpy.ok():
            elapsed = monotonic() - start
            terminal_result = self._terminal_deposit_result(
                goal_handle, elapsed, timeout_s
            )
            if terminal_result is not None:
                return terminal_result

            progress = self._publish_deposit_feedback(
                goal_handle, elapsed, nominal_duration
            )
            if progress >= 1.0:
                return self._succeeded_deposit_result(goal_handle, elapsed)

            sleep(self.loop_period_s)

        return self._shutdown_deposit_result(goal_handle, start)


def main(args=None):
    """Start the material action server node and spin until interrupted."""
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = MaterialActionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
