"""
Compact human-readable status dashboard for the nav stack.

Prints a single-line periodic summary instead of spamming logs:
  [DASH] odom:50Hz imu:100Hz pts:3.2Hz tag:YES ekf:OK costmap:READY pose:(-2.9,1.1,0.3rad)

Subscribes to key topics and aggregates rates/health into one line.
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool


class StatusDashboard(Node):
    """Compact periodic status summary for the navigation stack."""

    def __init__(self):
        """
        Initialise status dashboard node.

        Configures subscriptions, timing windows, and state used for periodic summaries.
        """
        super().__init__("status_dashboard")
        self.declare_parameter("period_sec", 5.0)
        period = self.get_parameter("period_sec").value

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self._counts = {"odom": 0, "imu": 0, "pts": 0, "tag": 0}
        self._costmap_ready = False
        self._last_pose = None
        self._last_tag_time = None
        self._window_start = self.get_clock().now()

        self.create_subscription(Odometry, "/odom", self._cb("odom"), sensor_qos)
        self.create_subscription(Imu, "/imu/data_raw", self._cb("imu"), sensor_qos)
        self.create_subscription(
            PointCloud2, "/camera_front/points", self._cb("pts"), sensor_qos
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/tag_pose", self._on_tag, sensor_qos
        )
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            Bool, "/nav/costmap_ready", self._on_costmap, status_qos
        )
        self.create_subscription(
            Odometry, "/odometry/filtered", self._on_filtered, sensor_qos
        )

        self.create_timer(period, self._print_status)

    def _cb(self, key):
        def callback(msg):
            self._counts[key] += 1
        return callback

    def _on_tag(self, msg):
        self._counts["tag"] += 1
        self._last_tag_time = self.get_clock().now()

    def _on_costmap(self, msg):
        self._costmap_ready = msg.data

    def _on_filtered(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._last_pose = (p.position.x, p.position.y, yaw)

    def _print_status(self):
        now = self.get_clock().now()
        dt = (now - self._window_start).nanoseconds / 1e9
        if dt < 0.1:
            return

        rates = {}
        for key in ("odom", "imu", "pts", "tag"):
            rates[key] = self._counts[key] / dt
            self._counts[key] = 0
        self._window_start = now

        if self._last_tag_time is None:
            tag_age_sec = float("inf")
        else:
            tag_age_sec = (now - self._last_tag_time).nanoseconds / 1e9
        tag_status = "YES" if tag_age_sec < 10.0 else "NO"
        costmap = "READY" if self._costmap_ready else "WAIT"

        if self._last_pose:
            x, y, yaw = self._last_pose
            pose_str = f"({x:.1f},{y:.1f},{math.degrees(yaw):.0f}deg)"
        else:
            pose_str = "(---)"

        odom_ok = rates["odom"] > 5.0
        imu_ok = rates["imu"] > 20.0
        pts_ok = rates["pts"] > 0.5
        all_ok = odom_ok and imu_ok and pts_ok

        self.get_logger().info(
            f"odom:{rates['odom']:.0f}Hz "
            f"imu:{rates['imu']:.0f}Hz "
            f"pts:{rates['pts']:.1f}Hz "
            f"tag:{tag_status}({rates['tag']:.1f}/s) "
            f"costmap:{costmap} "
            f"pose:{pose_str} "
            f"{'ALL OK' if all_ok else 'DEGRADED'}"
        )


def main(args=None):
    """
    Run status dashboard node.

    Initialise rclpy, spin the node, and shut down cleanly on exit.
    """
    rclpy.init(args=args)
    node = StatusDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
