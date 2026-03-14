"""Republish CameraInfo stamped to image timestamps for deterministic sync."""

from copy import deepcopy
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class CameraInfoSyncRepublisher(Node):
    """Publish a camera_info message for every image with matching stamp."""

    def __init__(self) -> None:
        """Initialise parameters, subscriptions, and output publisher."""
        super().__init__("camera_info_sync_republisher")

        self.declare_parameter("image_topic", "/camera_front/image")
        self.declare_parameter("camera_info_topic", "/camera_front/camera_info")
        self.declare_parameter(
            "camera_info_synced_topic", "/camera_front/camera_info_synced"
        )
        self.declare_parameter("max_info_age_sec", 1.0)
        self.declare_parameter("status_period_sec", 5.0)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.max_info_age = Duration(
            seconds=float(self.get_parameter("max_info_age_sec").value)
        )
        self.status_period_sec = float(self.get_parameter("status_period_sec").value)
        self.latest_info: Optional[CameraInfo] = None
        self.latest_info_receive_time = None
        self.image_count = 0
        self.info_count = 0
        self.pub_count = 0

        self.pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter("camera_info_synced_topic").value),
            sensor_qos,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self.on_camera_info,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            sensor_qos,
        )
        self.create_timer(self.status_period_sec, self.on_status_timer)

    def on_camera_info(self, msg: CameraInfo) -> None:
        """Cache the latest intrinsic/extrinsic camera calibration."""
        self.latest_info = msg
        self.latest_info_receive_time = self.get_clock().now()
        self.info_count += 1

    def on_image(self, msg: Image) -> None:
        """Publish a synced CameraInfo stamped to the incoming image."""
        self.image_count += 1
        if self.latest_info is None or self.latest_info_receive_time is None:
            return
        if self.get_clock().now() - self.latest_info_receive_time > self.max_info_age:
            return

        synced_info = deepcopy(self.latest_info)
        synced_info.header.stamp = msg.header.stamp
        if msg.header.frame_id:
            synced_info.header.frame_id = msg.header.frame_id
        self.pub.publish(synced_info)
        self.pub_count += 1

    def on_status_timer(self) -> None:
        """Emit low-rate diagnostics so sync starvation is visible in logs."""
        if self.info_count == 0:
            self.get_logger().warn(
                "[camera_info_sync] no camera_info received in last "
                f"{self.status_period_sec:.1f}s"
            )
        elif self.pub_count == 0 and self.image_count > 0:
            self.get_logger().warn(
                "[camera_info_sync] images seen but no synced camera_info published "
                f"in last {self.status_period_sec:.1f}s"
            )
        else:
            self.get_logger().info(
                "[camera_info_sync] window=%.1fs image=%d info=%d published=%d"
                % (
                    self.status_period_sec,
                    self.image_count,
                    self.info_count,
                    self.pub_count,
                )
            )

        self.image_count = 0
        self.info_count = 0
        self.pub_count = 0


def main(args=None) -> None:
    """Run the camera info sync republisher node."""
    rclpy.init(args=args)
    node = CameraInfoSyncRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
