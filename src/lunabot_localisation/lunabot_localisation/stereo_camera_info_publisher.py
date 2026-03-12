import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class StereoCameraInfoPublisher(Node):
    def __init__(self):
        super().__init__("stereo_camera_info_publisher")

        self.declare_parameter("width", 640)
        self.declare_parameter("height", 400)
        self.declare_parameter("hfov", 1.396263402)
        self.declare_parameter("baseline", 0.075)
        self.declare_parameter("left_image_topic", "/camera_front_left")
        self.declare_parameter("right_image_topic", "/camera_front_right")
        self.declare_parameter("left_camera_info_topic", "/camera_front_left/camera_info")
        self.declare_parameter("right_camera_info_topic", "/camera_front_right/camera_info")
        self.declare_parameter("left_frame_id", "camera_front_left_optical_frame")
        self.declare_parameter("right_frame_id", "camera_front_right_optical_frame")

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.hfov = float(self.get_parameter("hfov").value)
        self.baseline = float(self.get_parameter("baseline").value)
        self.left_frame_id = self.get_parameter("left_frame_id").value
        self.right_frame_id = self.get_parameter("right_frame_id").value

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.left_pub = self.create_publisher(
            CameraInfo,
            self.get_parameter("left_camera_info_topic").value,
            sensor_qos,
        )
        self.right_pub = self.create_publisher(
            CameraInfo,
            self.get_parameter("right_camera_info_topic").value,
            sensor_qos,
        )

        self.create_subscription(
            Image,
            self.get_parameter("left_image_topic").value,
            self.on_left_image,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            self.get_parameter("right_image_topic").value,
            self.on_right_image,
            sensor_qos,
        )

        self.fx = self.width / (2.0 * math.tan(self.hfov / 2.0))
        self.fy = self.fx
        self.cx = (self.width - 1) / 2.0
        self.cy = (self.height - 1) / 2.0

    def make_camera_info(self, header, frame_id: str, tx: float) -> CameraInfo:
        msg = CameraInfo()
        msg.header = header
        msg.header.frame_id = frame_id
        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0] * 5
        msg.k = [
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
        msg.r = [
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
        msg.p = [
            self.fx,
            0.0,
            self.cx,
            tx,
            0.0,
            self.fy,
            self.cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return msg

    def on_left_image(self, msg: Image) -> None:
        self.left_pub.publish(self.make_camera_info(msg.header, self.left_frame_id, 0.0))

    def on_right_image(self, msg: Image) -> None:
        self.right_pub.publish(
            self.make_camera_info(
                msg.header,
                self.right_frame_id,
                -self.fx * self.baseline,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
