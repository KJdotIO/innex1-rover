#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class MotionProbe(Node):
    """Probe node that commands velocity and watches odometry."""

    def __init__(self):
        """Initialise publishers/subscribers for motion checks."""
        super().__init__("sim_motion_probe")
        self._last_odom = None
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_cb, odom_qos
        )
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", cmd_qos)

    def _odom_cb(self, msg: Odometry) -> None:
        self._last_odom = msg

    @property
    def last_odom(self):
        """Return the latest odometry message, if any."""
        return self._last_odom

    def send_cmd(self, linear_x: float, angular_z: float) -> None:
        """Publish a single velocity command."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_pub.publish(msg)


def parse_args() -> argparse.Namespace:
    """Parse CLI options for the motion check."""
    parser = argparse.ArgumentParser(description="Verify robot moves after /cmd_vel")
    parser.add_argument("--timeout", type=float, default=25.0)
    parser.add_argument("--drive-seconds", type=float, default=3.0)
    parser.add_argument("--linear", type=float, default=0.25)
    parser.add_argument("--angular", type=float, default=0.0)
    parser.add_argument("--min-distance", type=float, default=0.03)
    return parser.parse_args()


def odom_xy(odom: Odometry):
    """Return planar (x, y) position from an odometry message."""
    return odom.pose.pose.position.x, odom.pose.pose.position.y


def main() -> int:
    """Run the motion check and return a shell-friendly exit code."""
    args = parse_args()
    rclpy.init()
    node = MotionProbe()

    deadline = time.time() + args.timeout
    while node.last_odom is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.last_odom is None:
        print("No /odom received before motion test")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    start_x, start_y = odom_xy(node.last_odom)

    drive_end = time.time() + args.drive_seconds
    while time.time() < drive_end:
        node.send_cmd(args.linear, args.angular)
        rclpy.spin_once(node, timeout_sec=0.05)

    for _ in range(10):
        node.send_cmd(0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.05)

    if node.last_odom is None:
        print("Lost /odom during motion test")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    end_x, end_y = odom_xy(node.last_odom)
    distance = math.hypot(end_x - start_x, end_y - start_y)

    node.destroy_node()
    rclpy.shutdown()

    print(f"Odom distance travelled: {distance:.4f} m")
    if distance < args.min_distance:
        threshold_msg = (
            "Movement check failed "
            f"(distance {distance:.4f} m < "
            f"threshold {args.min_distance:.4f} m)"
        )
        print(threshold_msg)
        return 1

    print("Movement check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
