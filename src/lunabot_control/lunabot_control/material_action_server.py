"""Stub action servers for excavation and deposition mission phases."""

from time import monotonic, sleep

import rclpy
from rclpy.action import ActionServer
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

        self._excavate_server = ActionServer(
            self,
            Excavate,
            "/mission/excavate",
            execute_callback=self.execute_excavate,
        )
        self._deposit_server = ActionServer(
            self,
            Deposit,
            "/mission/deposit",
            execute_callback=self.execute_deposit,
        )

        self.get_logger().info("Material action stubs ready")

    @property
    def loop_period_s(self):
        """Return the configured feedback publish period."""
        return float(self.get_parameter("loop_period_s").value)

    def _should_force_failure(self, action_name):
        """Return True when this action should intentionally fail."""
        value = self.get_parameter("force_failure_action").value
        configured = str(value).strip().lower()
        return configured == action_name

    def execute_excavate(self, goal_handle):
        """Run excavation stub with success, timeout, and failure paths."""
        nominal_duration = float(
            self.get_parameter("excavate_nominal_duration_s").value
        )
        timeout_s = float(goal_handle.request.timeout_s)
        start = monotonic()

        while rclpy.ok():
            elapsed = monotonic() - start

            if timeout_s > 0.0 and elapsed >= timeout_s:
                goal_handle.abort()
                result = Excavate.Result()
                result.success = False
                result.reason_code = 1
                result.failure_reason = "Excavation timeout reached"
                result.collected_mass_kg_estimate = 0.0
                result.duration_s = float(elapsed)
                return result

            if self._should_force_failure("excavate"):
                goal_handle.abort()
                result = Excavate.Result()
                result.success = False
                result.reason_code = 3
                result.failure_reason = "Forced failure for bench testing"
                result.collected_mass_kg_estimate = 0.0
                result.duration_s = float(elapsed)
                return result

            progress = min(elapsed / max(nominal_duration, 0.1), 1.0)

            feedback = Excavate.Feedback()
            feedback.phase = (
                0
                if progress < 0.2
                else (1 if progress < 0.35 else (2 if progress < 0.85 else 3))
            )
            feedback.elapsed_s = float(elapsed)
            feedback.fill_fraction_estimate = float(progress)
            feedback.excavation_motor_current_a = float(8.0 + 6.0 * progress)
            feedback.jam_detected = False
            feedback.estop_active = False
            goal_handle.publish_feedback(feedback)

            if progress >= 1.0:
                goal_handle.succeed()
                result = Excavate.Result()
                result.success = True
                result.reason_code = 0
                result.failure_reason = ""
                result.collected_mass_kg_estimate = 12.5
                result.duration_s = float(elapsed)
                return result

            sleep(self.loop_period_s)

        goal_handle.abort()
        result = Excavate.Result()
        result.success = False
        result.reason_code = 2
        result.failure_reason = "Node shutdown during excavation"
        result.collected_mass_kg_estimate = 0.0
        result.duration_s = float(monotonic() - start)
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

            if timeout_s > 0.0 and elapsed >= timeout_s:
                goal_handle.abort()
                result = Deposit.Result()
                result.success = False
                result.reason_code = 1
                result.failure_reason = "Deposition timeout reached"
                result.residual_fill_fraction_estimate = 1.0
                result.duration_s = float(elapsed)
                return result

            if self._should_force_failure("deposit"):
                goal_handle.abort()
                result = Deposit.Result()
                result.success = False
                result.reason_code = 3
                result.failure_reason = "Forced failure for bench testing"
                result.residual_fill_fraction_estimate = 1.0
                result.duration_s = float(elapsed)
                return result

            progress = min(elapsed / max(nominal_duration, 0.1), 1.0)

            if progress < 0.2:
                phase = 0
            elif progress < 0.45:
                phase = 1
            elif progress < 0.65:
                phase = 2
            elif progress < 0.85:
                phase = 3
            else:
                phase = 4

            feedback = Deposit.Feedback()
            feedback.phase = phase
            feedback.elapsed_s = float(elapsed)
            feedback.actuator_current_a = float(3.5 + 2.5 * progress)
            feedback.door_open = phase in (1, 2, 3)
            feedback.bed_raised = phase in (2, 3)
            feedback.estop_active = False
            goal_handle.publish_feedback(feedback)

            if progress >= 1.0:
                goal_handle.succeed()
                result = Deposit.Result()
                result.success = True
                result.reason_code = 0
                result.failure_reason = ""
                result.residual_fill_fraction_estimate = 0.05
                result.duration_s = float(elapsed)
                return result

            sleep(self.loop_period_s)

        goal_handle.abort()
        result = Deposit.Result()
        result.success = False
        result.reason_code = 2
        result.failure_reason = "Node shutdown during deposition"
        result.residual_fill_fraction_estimate = 1.0
        result.duration_s = float(monotonic() - start)
        return result


def main(args=None):
    """Start the material action server node and spin until interrupted."""
    rclpy.init(args=args)
    node = MaterialActionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
