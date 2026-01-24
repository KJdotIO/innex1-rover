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

"""Hazard detection node for processing depth camera point clouds."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class HazardDetectionNode(Node):
    """
    ROS 2 node for detecting hazards from depth camera data.

    Subscribes to depth camera point clouds and publishes filtered
    point clouds for navigation costmap integration.
    """

    def __init__(self):
        """Initialize the hazard detection node."""
        super().__init__('hazard_detection_node')
        self.publisher = self.create_publisher(
            PointCloud2, '/hazards/front', 10)

        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera_front/points',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        """
        Process incoming point cloud data.

        Args:
        ----
            msg: PointCloud2 message from the depth camera

        """
        self.publisher.publish(msg)


def main(args=None):
    """
    Run the hazard detection node.

    Args:
    ----
        args: Command line arguments (optional)

    """
    rclpy.init(args=args)
    node = HazardDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
