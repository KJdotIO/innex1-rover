"""Command velocity multiplexer for teleop/autonomy arbitration.

Subscribes to:
  /teleop/cmd_vel  — operator input (joystick or keyboard)
  /nav2/cmd_vel    — Nav2 controller output

Publishes to:
  /cmd_vel              — final velocity command to the wheels
  /mission/autonomy_mode — active source: "TELEOP" or "AUTONOMY"

Priority rule: if a non-zero teleop command was received within
``teleop_timeout_s`` seconds, teleop takes control and Nav2 is
suppressed. Otherwise Nav2 commands pass through.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from std_msgs.msg import String

MODE_TELEOP = "TELEOP"
MODE_AUTONOMY = "AUTONOMY"


class CmdVelMux(Node):
    """Priority mux: teleop overrides Nav2 when the operator is active."""

    def __init__(self):
        """Initialise subscriptions, publishers, and timeout parameter."""
        super().__init__("cmd_vel_mux")

        self.declare_parameter("teleop_timeout_s", 0.5)

        self._teleop_timeout_s = float(
            self.get_parameter("teleop_timeout_s").value
        )
        self._last_teleop_time: Time | None = None
        self._last_teleop_nonzero = False

        self.create_subscription(Twist, "/teleop/cmd_vel", self._teleop_cb, 10)
        self.create_subscription(Twist, "/nav2/cmd_vel", self._nav2_cb, 10)

        self._pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self._pub_mode = self.create_publisher(String, "/mission/autonomy_mode", 10)

        self.get_logger().info("cmd_vel_mux online — teleop priority active")

    def _teleop_active(self) -> bool:
        """Return True if a non-zero teleop command arrived recently."""
        if self._last_teleop_time is None or not self._last_teleop_nonzero:
            return False
        now = self.get_clock().now()
        elapsed = (now - self._last_teleop_time).nanoseconds * 1e-9
        return elapsed < self._teleop_timeout_s

    @staticmethod
    def _is_nonzero(msg: Twist) -> bool:
        """Return True if any velocity component is non-zero."""
        lin = msg.linear
        ang = msg.angular
        return any(v != 0.0 for v in (lin.x, lin.y, lin.z, ang.x, ang.y, ang.z))

    def _teleop_cb(self, msg: Twist) -> None:
        """Record teleop arrival time and forward if operator is active."""
        self._last_teleop_nonzero = self._is_nonzero(msg)
        self._last_teleop_time = self.get_clock().now()

        if self._teleop_active():
            self._pub_cmd_vel.publish(msg)
            self._publish_mode(MODE_TELEOP)

    def _nav2_cb(self, msg: Twist) -> None:
        """Forward Nav2 command only when teleop is not active."""
        if self._teleop_active():
            return
        self._pub_cmd_vel.publish(msg)
        self._publish_mode(MODE_AUTONOMY)

    def _publish_mode(self, mode: str) -> None:
        """Publish the current autonomy mode string."""
        mode_msg = String()
        mode_msg.data = mode
        self._pub_mode.publish(mode_msg)


def main(args=None):
    """Start the cmd_vel_mux node and spin until interrupted."""
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
