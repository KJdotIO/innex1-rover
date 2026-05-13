# Copyright 2026 University of Leicester
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Republish CameraInfo with image timestamps for simulation-only consumers."""

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraInfoStampAligner(Node):
    """Align simulated CameraInfo stamps to the latest front-camera image."""

    def __init__(self):
        """Create subscriptions and the aligned CameraInfo publisher."""
        super().__init__("camera_info_stamp_aligner")

        self.declare_parameter("image_topic", "/camera_front/image")
        self.declare_parameter("camera_info_topic", "/camera_front/camera_info")
        self.declare_parameter(
            "aligned_camera_info_topic",
            "/camera_front/camera_info_synced",
        )

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        aligned_camera_info_topic = self.get_parameter(
            "aligned_camera_info_topic"
        ).value

        self.latest_camera_info: CameraInfo | None = None
        self.aligned_pub = self.create_publisher(
            CameraInfo,
            aligned_camera_info_topic,
            10,
        )
        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Aligning {camera_info_topic} stamps to {image_topic} on "
            f"{aligned_camera_info_topic}"
        )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """Store the most recent camera calibration message."""
        self.latest_camera_info = msg

    def _image_callback(self, msg: Image) -> None:
        """Publish camera info with the current image timestamp."""
        if self.latest_camera_info is None:
            return

        aligned_info = deepcopy(self.latest_camera_info)
        aligned_info.header.stamp = msg.header.stamp
        aligned_info.header.frame_id = msg.header.frame_id
        self.aligned_pub.publish(aligned_info)


def main(args=None) -> None:
    """Run the camera-info stamp aligner."""
    rclpy.init(args=args)
    node = CameraInfoStampAligner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
