"""Republish RGB-D streams with monotonic timestamps and stale-frame dropping."""

from __future__ import annotations

import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class RGBDStreamRepublisher(Node):
    """Normalize RGB-D timestamps for downstream sync-sensitive nodes."""

    def __init__(self) -> None:
        super().__init__("rgbd_stream_republisher")

        self.declare_parameter("input_image_topic", "/camera_front/image")
        self.declare_parameter("input_depth_topic", "/camera_front/depth_image")
        self.declare_parameter("input_camera_info_topic", "/camera_front/camera_info")
        self.declare_parameter("output_image_topic", "/camera_front/image_sync")
        self.declare_parameter("output_depth_topic", "/camera_front/depth_image_sync")
        self.declare_parameter("output_camera_info_topic", "/camera_front/camera_info_sync")
        self.declare_parameter("max_input_age_sec", 0.0)
        self.declare_parameter("status_period_sec", 5.0)
        self.declare_parameter("publish_info_on_depth", False)

        self.max_input_age_sec = float(self.get_parameter("max_input_age_sec").value)
        status_period_sec = float(self.get_parameter("status_period_sec").value)
        self.publish_info_on_depth = bool(
            self.get_parameter("publish_info_on_depth").value
        )

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.image_pub = self.create_publisher(
            Image,
            str(self.get_parameter("output_image_topic").value),
            sensor_qos,
        )
        self.depth_pub = self.create_publisher(
            Image,
            str(self.get_parameter("output_depth_topic").value),
            sensor_qos,
        )
        self.info_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter("output_camera_info_topic").value),
            sensor_qos,
        )

        self.latest_info: CameraInfo | None = None
        self.last_stamp_ns = 0
        self.expected_width: int | None = None
        self.expected_height: int | None = None

        self.in_image = 0
        self.in_depth = 0
        self.dropped_image = 0
        self.dropped_depth = 0
        self.pub_image = 0
        self.pub_depth = 0
        self.pub_info = 0
        self.info_dim_mismatch = 0
        self.stream_dim_mismatch = 0

        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("input_camera_info_topic").value),
            self.on_camera_info,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("input_image_topic").value),
            self.on_image,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("input_depth_topic").value),
            self.on_depth,
            sensor_qos,
        )

        self.create_timer(status_period_sec, self.log_status)

    def on_camera_info(self, msg: CameraInfo) -> None:
        """Cache latest camera info for synchronized republishing."""
        self.latest_info = msg

    def on_image(self, msg: Image) -> None:
        """Republish color image with corrected timestamp."""
        self.in_image += 1
        if not self._accept_dimensions(msg.width, msg.height):
            self.dropped_image += 1
            return
        stamp = self._next_stamp(msg)
        if stamp is None:
            self.dropped_image += 1
            return

        out = copy.deepcopy(msg)
        out.header.stamp = stamp
        self.image_pub.publish(out)
        self.pub_image += 1
        self._publish_info(stamp, out.width, out.height)

    def on_depth(self, msg: Image) -> None:
        """Republish depth image with corrected timestamp."""
        self.in_depth += 1
        if not self._accept_dimensions(msg.width, msg.height):
            self.dropped_depth += 1
            return
        stamp = self._next_stamp(msg)
        if stamp is None:
            self.dropped_depth += 1
            return

        out = copy.deepcopy(msg)
        out.header.stamp = stamp
        self.depth_pub.publish(out)
        self.pub_depth += 1
        if self.publish_info_on_depth:
            self._publish_info(stamp, out.width, out.height)

    def _next_stamp(self, msg: Image):
        """Return a fresh monotonic stamp or None when input frame is stale."""
        now = self.get_clock().now()
        input_stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        )
        now_ns = now.nanoseconds

        if self.max_input_age_sec > 0.0 and input_stamp_ns > 0:
            age_sec = (now_ns - input_stamp_ns) / 1e9
            if age_sec > self.max_input_age_sec:
                return None

        if now_ns <= self.last_stamp_ns:
            now_ns = self.last_stamp_ns + 1

        self.last_stamp_ns = now_ns
        return rclpy.time.Time(nanoseconds=now_ns).to_msg()

    def _accept_dimensions(self, width: int, height: int) -> bool:
        """Lock output stream dimensions and reject mismatched frame bursts."""
        if self.expected_width is None or self.expected_height is None:
            self.expected_width = int(width)
            self.expected_height = int(height)
            return True
        if int(width) == self.expected_width and int(height) == self.expected_height:
            return True
        self.stream_dim_mismatch += 1
        return False

    def _publish_info(self, stamp, expected_width: int, expected_height: int) -> None:
        """Republish camera info with the exact image/depth stamp."""
        if self.latest_info is None:
            return
        if (
            int(self.latest_info.width) != int(expected_width)
            or int(self.latest_info.height) != int(expected_height)
        ):
            self.info_dim_mismatch += 1
            return

        info = copy.deepcopy(self.latest_info)
        info.header.stamp = stamp
        self.info_pub.publish(info)
        self.pub_info += 1

    def log_status(self) -> None:
        """Emit compact per-window status counters."""
        self.get_logger().info(
            "[rgbd_stream] expected=%sx%s in_image=%d in_depth=%d "
            "dropped_image=%d dropped_depth=%d pub_image=%d pub_depth=%d "
            "pub_info=%d info_dim_mismatch=%d stream_dim_mismatch=%d"
            % (
                str(self.expected_width),
                str(self.expected_height),
                self.in_image,
                self.in_depth,
                self.dropped_image,
                self.dropped_depth,
                self.pub_image,
                self.pub_depth,
                self.pub_info,
                self.info_dim_mismatch,
                self.stream_dim_mismatch,
            )
        )
        self.in_image = 0
        self.in_depth = 0
        self.dropped_image = 0
        self.dropped_depth = 0
        self.pub_image = 0
        self.pub_depth = 0
        self.pub_info = 0
        self.info_dim_mismatch = 0
        self.stream_dim_mismatch = 0


def main(args=None) -> None:
    """Run RGB-D stream republisher."""
    rclpy.init(args=args)
    node = RGBDStreamRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
