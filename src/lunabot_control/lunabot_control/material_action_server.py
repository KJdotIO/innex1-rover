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
        if dump_duration_s < 0.0:
            return False
        return True

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

    def execute_deposit(self, goal_handle):
        """Run deposition stub with success, timeout, and failure paths."""
        nominal_duration = float(self.get_parameter("deposit_nominal_duration_s").value)
        dump_duration = float(goal_handle.request.dump_duration_s)
        if dump_duration > 0.0:
            nominal_duration = max(nominal_duration, dump_duration + 2.0)

        timeout_s = float(goal_handle.request.timeout_s)
        start = monotonic()

        while rclpy.ok():
            elapsed = monotonic() - start

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._deposit_result(
                    False,
                    Deposit.Result.REASON_CANCELED,
                    "Deposit goal canceled by client",
                    1.0,
                    elapsed,
                )

            if timeout_s > 0.0 and elapsed >= timeout_s:
                goal_handle.abort()
                return self._deposit_result(
                    False,
                    Deposit.Result.REASON_TIMEOUT,
                    "Deposition timeout reached",
                    1.0,
                    elapsed,
                )

            if self._should_force_failure("deposit"):
                goal_handle.abort()
                return self._deposit_result(
                    False,
                    Deposit.Result.REASON_FORCED_FAILURE,
                    "Forced failure for bench testing",
                    1.0,
                    elapsed,
                )

            progress = min(elapsed / max(nominal_duration, 0.1), 1.0)

            feedback = Deposit.Feedback()
            if progress < 0.2:
                feedback.phase = Deposit.Feedback.PHASE_PRECHECK
            elif progress < 0.45:
                feedback.phase = Deposit.Feedback.PHASE_OPENING
            elif progress < 0.65:
                feedback.phase = Deposit.Feedback.PHASE_RAISING
            elif progress < 0.85:
                feedback.phase = Deposit.Feedback.PHASE_DUMPING
            else:
                feedback.phase = Deposit.Feedback.PHASE_CLOSING
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

            if progress >= 1.0:
                goal_handle.succeed()
                return self._deposit_result(
                    True,
                    Deposit.Result.REASON_SUCCESS,
                    "",
                    0.05,
                    elapsed,
                )

            sleep(self.loop_period_s)

        if goal_handle.is_active:
            goal_handle.abort()
        return self._deposit_result(
            False,
            Deposit.Result.REASON_SHUTDOWN,
            "Node shutdown during deposition",
            1.0,
            monotonic() - start,
        )


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
