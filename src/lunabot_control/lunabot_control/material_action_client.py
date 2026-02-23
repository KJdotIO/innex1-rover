"""Simple client that exercises excavation then deposition action calls."""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from lunabot_interfaces.action import Deposit, Excavate


class MaterialActionClient(Node):
    """Send a basic excavate/deposit sequence for bench and CI smoke checks."""

    def __init__(self):
        """Initialise action clients and configurable goal parameters."""
        super().__init__("material_action_client")

        self.declare_parameter("excavate_timeout_s", 12.0)
        self.declare_parameter("deposit_timeout_s", 10.0)
        self.declare_parameter("dump_duration_s", 3.0)

        self._excavate_client = ActionClient(
            self,
            Excavate,
            "/mission/excavate",
        )
        self._deposit_client = ActionClient(self, Deposit, "/mission/deposit")

    def run_sequence(self):
        """Run one excavation request followed by one deposition request."""
        if not self._excavate_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Excavate action server not available")
            return 1

        if not self._deposit_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Deposit action server not available")
            return 1

        excavate_goal = Excavate.Goal()
        excavate_goal.mode = 0
        excavate_goal.timeout_s = float(self.get_parameter("excavate_timeout_s").value)
        excavate_goal.target_fill_fraction = 0.8
        excavate_goal.max_drive_speed_mps = 0.2

        self.get_logger().info("Sending excavation goal")
        send_goal_future = self._excavate_client.send_goal_async(excavate_goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        excavate_handle = send_goal_future.result()
        if not excavate_handle.accepted:
            self.get_logger().error("Excavation goal rejected")
            return 1

        excavate_result_future = excavate_handle.get_result_async()
        rclpy.spin_until_future_complete(self, excavate_result_future)
        excavate_result = excavate_result_future.result().result
        if not excavate_result.success:
            self.get_logger().error(
                f"Excavation failed ({excavate_result.reason_code}): "
                f"{excavate_result.failure_reason}"
            )
            return 1

        deposit_goal = Deposit.Goal()
        deposit_goal.mode = 0
        deposit_goal.timeout_s = float(self.get_parameter("deposit_timeout_s").value)
        deposit_goal.dump_duration_s = float(
            self.get_parameter("dump_duration_s").value
        )
        deposit_goal.require_close_after_dump = True

        self.get_logger().info("Sending deposition goal")
        send_goal_future = self._deposit_client.send_goal_async(deposit_goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        deposit_handle = send_goal_future.result()
        if not deposit_handle.accepted:
            self.get_logger().error("Deposition goal rejected")
            return 1

        deposit_result_future = deposit_handle.get_result_async()
        rclpy.spin_until_future_complete(self, deposit_result_future)
        deposit_result = deposit_result_future.result().result
        if not deposit_result.success:
            self.get_logger().error(
                f"Deposition failed ({deposit_result.reason_code}): "
                f"{deposit_result.failure_reason}"
            )
            return 1

        self.get_logger().info("Material action sequence completed")
        return 0


def main(args=None):
    """Run a one-shot material action sequence and exit with status code."""
    rclpy.init(args=args)
    node = MaterialActionClient()
    try:
        status = node.run_sequence()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if status != 0:
        raise SystemExit(status)


if __name__ == "__main__":
    main()
