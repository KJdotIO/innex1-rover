"""Stub action servers for excavation and deposition mission phases."""

from time import monotonic, sleep

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from lunabot_interfaces.action import Deposit, Excavate


class MaterialActionServer(Node):
    """Provide action stubs for excavation and deposition."""

    def __init__(self):
        """Initialise action servers and simulation parameters."""
        super().__init__("material_action_server")

        self.declare_parameter("excavate_nominal_duration_s", 8.0)
        self.declare_parameter("deposit_nominal_duration_s", 5.0)
        self.declare_parameter("loop_period_s", 0.2)
        self.declare_parameter("force_failure_action", "")

        self._callback_group = ReentrantCallbackGroup()

        self._excavate_server = ActionServer(
            self,
            Excavate,
            "/mission/excavate",
            execute_callback=self.execute_excavate,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )
        self._deposit_server = ActionServer(
            self,
            Deposit,
            "/mission/deposit",
            execute_callback=self.execute_deposit,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info("Material action stubs ready")

    def destroy_node(self):
        """Destroy action server handles before node teardown."""
        self._excavate_server.destroy()
        self._deposit_server.destroy()
        super().destroy_node()

    def goal_callback(self, _goal_request):
        """Accept each goal request for bench and integration testing."""
        return GoalResponse.ACCEPT

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
    def _excavate_result(success, reason_code, reason, mass_kg, duration_s):
        """Build an Excavate result message."""
        result = Excavate.Result()
        result.success = bool(success)
        result.reason_code = int(reason_code)
        result.failure_reason = str(reason)
        result.collected_mass_kg_estimate = float(mass_kg)
        result.duration_s = float(duration_s)
        return result

    @staticmethod
    def _deposit_result(success, reason_code, reason, residual_fill, duration_s):
        """Build a Deposit result message."""
        result = Deposit.Result()
        result.success = bool(success)
        result.reason_code = int(reason_code)
        result.failure_reason = str(reason)
        result.residual_fill_fraction_estimate = float(residual_fill)
        result.duration_s = float(duration_s)
        return result

    def execute_excavate(self, goal_handle):
        """Run excavation stub with success, timeout, and failure paths."""
        nominal_duration = float(
            self.get_parameter("excavate_nominal_duration_s").value
        )
        timeout_s = float(goal_handle.request.timeout_s)
        start = monotonic()

        while rclpy.ok():
            elapsed = monotonic() - start

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._excavate_result(
                    False,
                    Excavate.Result.REASON_CANCELED,
                    "Excavation goal canceled by client",
                    0.0,
                    elapsed,
                )

            if timeout_s > 0.0 and elapsed >= timeout_s:
                goal_handle.abort()
                return self._excavate_result(
                    False,
                    Excavate.Result.REASON_TIMEOUT,
                    "Excavation timeout reached",
                    0.0,
                    elapsed,
                )

            if self._should_force_failure("excavate"):
                goal_handle.abort()
                return self._excavate_result(
                    False,
                    Excavate.Result.REASON_FORCED_FAILURE,
                    "Forced failure for bench testing",
                    0.0,
                    elapsed,
                )

            progress = min(elapsed / max(nominal_duration, 0.1), 1.0)

            feedback = Excavate.Feedback()
            if progress < 0.2:
                feedback.phase = Excavate.Feedback.PHASE_PRECHECK
            elif progress < 0.35:
                feedback.phase = Excavate.Feedback.PHASE_SPINUP
            elif progress < 0.85:
                feedback.phase = Excavate.Feedback.PHASE_DIGGING
            else:
                feedback.phase = Excavate.Feedback.PHASE_RETRACT
            feedback.elapsed_s = float(elapsed)
            feedback.fill_fraction_estimate = float(progress)
            feedback.excavation_motor_current_a = float(8.0 + 6.0 * progress)
            feedback.jam_detected = False
            feedback.estop_active = False
            goal_handle.publish_feedback(feedback)

            if progress >= 1.0:
                goal_handle.succeed()
                return self._excavate_result(
                    True,
                    Excavate.Result.REASON_SUCCESS,
                    "",
                    12.5,
                    elapsed,
                )

            sleep(self.loop_period_s)

        if goal_handle.is_active:
            goal_handle.abort()
        return self._excavate_result(
            False,
            Excavate.Result.REASON_SHUTDOWN,
            "Node shutdown during excavation",
            0.0,
            monotonic() - start,
        )

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
