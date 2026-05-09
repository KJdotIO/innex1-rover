"""E-stop node: latches /safety/estop into /safety/motion_inhibit."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

# TRANSIENT_LOCAL so late subscribers (e.g. motor controllers) receive the current
# inhibit state immediately on connect rather than waiting for the next e-stop event.
_INHIBIT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class EstopNode(Node):
    """Subscribe to the hardware e-stop and publish a motion-inhibit signal."""

    def __init__(self):
        """Initialise publishers, subscribers, and internal state."""
        super().__init__("estop_node")

        self._estop_active = False
        self._inhibited = False
        self._reset_required = False

        self._motion_inhibit_pub = self.create_publisher(
            Bool,
            "/safety/motion_inhibit",
            _INHIBIT_QOS,
        )

        self._estop_sub = self.create_subscription(
            Bool,
            "/safety/estop",
            self._estop_callback,
            10,
        )

        self._reset_sub = self.create_subscription(
            Bool,
            "/safety/reset_motion_inhibit",
            self._reset_callback,
            10,
        )

        # Publish initial state so subscribers joining before any e-stop event
        # receive a well-defined value.
        self._publish_inhibit()
        self.get_logger().info("estop_node started; waiting for /safety/estop")

    def _publish_inhibit(self) -> None:
        """Publish the current motion-inhibit state."""
        out = Bool()
        out.data = self._inhibited
        self._motion_inhibit_pub.publish(out)

    def _estop_callback(self, msg: Bool) -> None:
        """Handle incoming e-stop signal and latch motion inhibit until reset."""
        new_state = bool(msg.data)

        if new_state == self._estop_active:
            self._publish_inhibit()
            return

        self._estop_active = new_state
        if self._estop_active:
            self._reset_required = True
            self._inhibited = True
            self.get_logger().warn(
                "E-stop active; motion inhibited until reset"
            )
        else:
            if self._reset_required:
                self.get_logger().info(
                    "E-stop cleared; publish /safety/reset_motion_inhibit "
                    "before motion is allowed"
                )
            else:
                self._inhibited = False

        self._publish_inhibit()

    def _reset_callback(self, msg: Bool) -> None:
        """Clear a latched motion inhibit once the E-stop input is clear."""
        if not msg.data:
            return

        if self._estop_active:
            self.get_logger().warn(
                "Motion-inhibit reset ignored while E-stop is active"
            )
            self._publish_inhibit()
            return

        if self._reset_required or self._inhibited:
            self.get_logger().info("Motion inhibit reset; motion allowed")

        self._reset_required = False
        self._inhibited = False
        self._publish_inhibit()

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
