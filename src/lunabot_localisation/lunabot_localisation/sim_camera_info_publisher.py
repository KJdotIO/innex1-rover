"""Publish deterministic CameraInfo messages aligned to sim image stamps."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class SimCameraInfoPublisher(Node):
    """Publish synthetic intrinsics using the incoming image timestamp."""

    def __init__(self) -> None:
        super().__init__("sim_camera_info_publisher")

        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("hfov", 1.57)
        self.declare_parameter("image_topic", "/camera_front/image")
        self.declare_parameter("camera_info_topic", "/camera_front/camera_info")
        self.declare_parameter("frame_id", "camera_front_optical_frame")

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.hfov = float(self.get_parameter("hfov").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.publisher = self.create_publisher(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            sensor_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            sensor_qos,
        )

        self.fx = self.width / (2.0 * math.tan(self.hfov / 2.0))
        self.fy = self.fx
        self.cx = (self.width - 1) / 2.0
        self.cy = (self.height - 1) / 2.0

    def on_image(self, msg: Image) -> None:
        """Publish CameraInfo with the same timestamp as the image."""
        camera_info = CameraInfo()
        camera_info.header = msg.header
        camera_info.header.frame_id = self.frame_id
        camera_info.width = self.width
        camera_info.height = self.height
        camera_info.distortion_model = "plumb_bob"
        camera_info.d = [0.0] * 5
        camera_info.k = [
            self.fx,
            0.0,
            self.cx,
            0.0,
            self.fy,
            self.cy,
            0.0,
            0.0,
            1.0,
        ]
        camera_info.r = [
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        camera_info.p = [
            self.fx,
            0.0,
            self.cx,
            0.0,
            0.0,
            self.fy,
            self.cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        self.publisher.publish(camera_info)


def main(args=None) -> None:
    """Run the sim CameraInfo publisher node."""
    rclpy.init(args=args)
    node = SimCameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
