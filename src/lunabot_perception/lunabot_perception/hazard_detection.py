"""Hazard detection node for processing depth camera point clouds."""

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class HazardDetectionNode(Node):
    """
    ROS 2 node for detecting hazards from depth camera data.

    Subscribes to depth camera point clouds and publishes filtered
    point clouds for navigation costmap integration.
    """

    def __init__(self):
        """Initialize the hazard detection node."""
        super().__init__("hazard_detection_node")
        self.publisher = self.create_publisher(PointCloud2, "/hazards/front", 10)

        self.subscription = self.create_subscription(
            PointCloud2, "/camera_front/points", self.listener_callback, 10
        )

    def listener_callback(self, msg):
        """
        Process incoming point cloud data.

        msg: PointCloud2 message from the depth camera.
        """
        extracted_points_generator = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        points_list = list(extracted_points_generator)
        extracted_points = np.array(
            [[p[0], p[1], p[2]] for p in points_list], dtype=np.float64
        )
        if len(extracted_points) == 0:
            return

        # Filter out infinite and NaN values
        valid_mask = np.all(np.isfinite(extracted_points), axis=1)
        extracted_points = extracted_points[valid_mask]

        if len(extracted_points) == 0:
            self.get_logger().warn("No valid points after filtering infinities")
            return

        # Creating point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(extracted_points)

        # Downsampled point cloud
        pcd_downsampled = pcd.voxel_down_sample(voxel_size=0.05)
        pcd_cleaned, outlier_indices = pcd_downsampled.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=2.0
        )

        # Segmented plane
        plane_model, inliers = pcd_cleaned.segment_plane(
            distance_threshold=0.05, ransac_n=3, num_iterations=1000
        )

        # Detecting boulders, craters and other obstacles
        pcd_obstacles = pcd_cleaned.select_by_index(inliers, invert=True)

        # Publishing point cloud of obstacles to ros2 topic
        points = np.array(pcd_obstacles.points)
        pc2_message = point_cloud2.create_cloud_xyz32(msg.header, points)
        self.publisher.publish(pc2_message)


def main(args=None):
    """
    Run the hazard detection node.

    args: Command line arguments (optional).
    """
    rclpy.init(args=args)
    node = HazardDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
