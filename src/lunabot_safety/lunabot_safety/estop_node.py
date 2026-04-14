"""E-stop node: bridges /safety/estop hardware signal to /safety/motion_inhibit."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class EstopNode(Node):
    """Subscribe to the hardware e-stop and publish a motion-inhibit signal."""

    def __init__(self):
        """Initialise publishers, subscribers, and internal state."""
        super().__init__("estop_node")

        self._inhibited = False

        self._motion_inhibit_pub = self.create_publisher(
            Bool,
            "/safety/motion_inhibit",
            10,
        )

        self._estop_sub = self.create_subscription(
            Bool,
            "/safety/estop",
            self._estop_callback,
            10,
        )

        self.get_logger().info("estop_node started — waiting for /safety/estop")

    def _estop_callback(self, msg: Bool) -> None:
        """Handle incoming e-stop signal and propagate motion-inhibit state."""
        new_state = msg.data

        if new_state != self._inhibited:
            if new_state:
                self.get_logger().info("E-stop ACTIVE — motion inhibited")
            else:
                self.get_logger().info("E-stop cleared — motion allowed")
            self._inhibited = new_state

        out = Bool()
        out.data = self._inhibited
        self._motion_inhibit_pub.publish(out)

    def destroy_node(self):
        """Clean up before shutdown."""
        super().destroy_node()


def main(args=None):
    """Entry point for the estop_node executable."""
    rclpy.init(args=args)
    node = EstopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
