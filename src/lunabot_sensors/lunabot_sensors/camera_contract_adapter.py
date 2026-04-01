"""Normalise OAK camera outputs to the repo's public front-camera contract."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


class CameraContractAdapter(Node):
    """Relay upstream camera topics onto the stable rover contract."""

    def __init__(self) -> None:
        super().__init__("camera_contract_adapter")

        self.declare_parameter("rgb_image_topic", "/oak/rgb/image_raw")
        self.declare_parameter("rgb_camera_info_topic", "/oak/rgb/camera_info")
        self.declare_parameter("depth_image_topic", "/oak/stereo/image_raw")
        self.declare_parameter("point_cloud_topic", "/oak/points")
        self.declare_parameter("optical_frame_id", "camera_front_optical_frame")
        self.declare_parameter("point_cloud_frame_id", "")

        self.optical_frame_id = str(self.get_parameter("optical_frame_id").value)
        self.point_cloud_frame_id = str(
            self.get_parameter("point_cloud_frame_id").value
        ).strip()

        self.image_pub = self.create_publisher(
            Image, "/camera_front/image", qos_profile_sensor_data
        )
        self.camera_info_pub = self.create_publisher(
            CameraInfo, "/camera_front/camera_info", qos_profile_sensor_data
        )
        self.depth_pub = self.create_publisher(
            Image, "/camera_front/depth_image", qos_profile_sensor_data
        )
        self.points_pub = self.create_publisher(
            PointCloud2, "/camera_front/points", qos_profile_sensor_data
        )

        self._subscriptions = [
            self.create_subscription(
                Image,
                str(self.get_parameter("rgb_image_topic").value),
                self._on_rgb_image,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                CameraInfo,
                str(self.get_parameter("rgb_camera_info_topic").value),
                self._on_camera_info,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Image,
                str(self.get_parameter("depth_image_topic").value),
                self._on_depth_image,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("point_cloud_topic").value),
                self._on_point_cloud,
                qos_profile_sensor_data,
            ),
        ]

        self.get_logger().info(
            "Normalising camera topics onto /camera_front/* with frame "
            f"'{self.optical_frame_id}'"
        )

    def _normalise_message(self, msg, frame_id: str):
        msg.header.frame_id = frame_id
        return msg

    def _on_rgb_image(self, msg: Image) -> None:
        self.image_pub.publish(self._normalise_message(msg, self.optical_frame_id))

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_info_pub.publish(
            self._normalise_message(msg, self.optical_frame_id)
        )

    def _on_depth_image(self, msg: Image) -> None:
        self.depth_pub.publish(self._normalise_message(msg, self.optical_frame_id))

    def _on_point_cloud(self, msg: PointCloud2) -> None:
        if self.point_cloud_frame_id:
            msg = self._normalise_message(msg, self.point_cloud_frame_id)
        self.points_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    """Run the camera contract adapter node."""
    rclpy.init(args=args)
    node = CameraContractAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
