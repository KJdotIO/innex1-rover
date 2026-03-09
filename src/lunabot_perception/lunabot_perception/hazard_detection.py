"""Hazard detection node for positive and negative obstacle marking."""

from __future__ import annotations

from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2


class HazardDetectionNode(Node):
    """Detect positive and negative hazards from front depth point clouds."""

    def __init__(self) -> None:
        """Initialize publishers, subscriber, and tuning parameters."""
        super().__init__("hazard_detection")

        self.declare_parameter("input_topic", "/camera_front/points")
        self.declare_parameter("positive_topic", "/hazards/front")
        self.declare_parameter("negative_topic", "/hazards/front_negative")
        self.declare_parameter(
            "negative_debug_mask_topic", "/hazards/debug/negative_mask"
        )
        self.declare_parameter(
            "negative_debug_edges_topic", "/hazards/debug/depth_edges"
        )
        self.declare_parameter("publish_debug_images", True)
        self.declare_parameter("min_range", 0.1)
        self.declare_parameter("max_range", 5.0)
        self.declare_parameter("downsample_step", 3)
        self.declare_parameter("ransac_iterations", 120)
        self.declare_parameter("plane_distance_threshold", 0.05)
        self.declare_parameter("positive_height_threshold", 0.10)
        self.declare_parameter("positive_max_height", 0.50)
        self.declare_parameter("negative_discontinuity_threshold", 0.25)
        self.declare_parameter("negative_gradient_threshold", 0.18)
        self.declare_parameter("negative_min_row_ratio", 0.2)
        self.declare_parameter("min_points_per_cloud", 20)

        input_topic = self._param_str("input_topic")
        configured_positive_topic = self._param_str("positive_topic")
        negative_topic = self._param_str("negative_topic")

        self.publish_debug_images = self._param_bool("publish_debug_images")
        self.min_range = self._param_float("min_range")
        self.max_range = self._param_float("max_range")
        self.downsample_step = max(1, self._param_int("downsample_step"))
        self.ransac_iterations = max(10, self._param_int("ransac_iterations"))
        self.plane_distance_threshold = self._param_float("plane_distance_threshold")
        self.positive_height_threshold = self._param_float("positive_height_threshold")
        self.positive_max_height = self._param_float("positive_max_height")
        self.negative_discontinuity_threshold = self._param_float(
            "negative_discontinuity_threshold"
        )
        self.negative_gradient_threshold = self._param_float(
            "negative_gradient_threshold"
        )
        self.negative_min_row_ratio = self._param_float("negative_min_row_ratio")
        self.min_points_per_cloud = self._param_int("min_points_per_cloud")

        if configured_positive_topic != "/hazards/front":
            self.get_logger().warn(
                "positive_topic is overridden but interface contract requires "
                "publishing on /hazards/front"
            )

        self.positive_publisher = self.create_publisher(
            PointCloud2,
            "/hazards/front",
            10,
        )
        self.negative_publisher = self.create_publisher(PointCloud2, negative_topic, 10)

        self.mask_publisher = None
        self.edges_publisher = None
        if self.publish_debug_images:
            self.mask_publisher = self.create_publisher(
                Image,
                self._param_str("negative_debug_mask_topic"),
                10,
            )
            self.edges_publisher = self.create_publisher(
                Image,
                self._param_str("negative_debug_edges_topic"),
                10,
            )

        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.listener_callback,
            10,
        )

    def _param_str(self, name: str) -> str:
        """Return a declared parameter as string."""
        value = self.get_parameter(name).value
        return "" if value is None else str(value)

    def _param_float(self, name: str) -> float:
        """Return a declared parameter as float."""
        value: Any = self.get_parameter(name).value
        return float(value)

    def _param_int(self, name: str) -> int:
        """Return a declared parameter as int."""
        value: Any = self.get_parameter(name).value
        return int(value)

    def _param_bool(self, name: str) -> bool:
        """Return a declared parameter as bool."""
        return bool(self.get_parameter(name).value)

    def listener_callback(self, msg: PointCloud2) -> None:
        """Process input point cloud and publish hazard outputs."""
        cloud = self._pointcloud2_to_numpy(msg)
        if cloud is None:
            return

        cloud_xyz, organized, valid_depth = cloud
        valid_points = cloud_xyz[valid_depth]

        if valid_points.shape[0] < self.min_points_per_cloud:
            return

        positive_points = self._detect_positive_hazards(valid_points)
        negative_mask = self._detect_negative_hazard_mask(organized, valid_depth)
        negative_points = organized[negative_mask]

        self._publish_cloud(self.positive_publisher, msg.header, positive_points)
        self._publish_cloud(self.negative_publisher, msg.header, negative_points)

        if self.publish_debug_images and self.mask_publisher and self.edges_publisher:
            self._publish_debug_images(msg, organized, valid_depth, negative_mask)

    def _pointcloud2_to_numpy(
        self, msg: PointCloud2
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        """Convert PointCloud2 to flat and organized numpy arrays."""
        if msg.width == 0 or msg.height == 0:
            return None

        points = np.array(
            list(
                point_cloud2.read_points(
                    msg,
                    field_names=["x", "y", "z"],
                    skip_nans=False,
                )
            ),
            dtype=np.float32,
        )

        expected_size = msg.width * msg.height
        if points.shape[0] != expected_size:
            return None

        organized = points.reshape((msg.height, msg.width, 3))
        depth = organized[:, :, 2]

        valid_depth = np.isfinite(depth)
        valid_depth &= depth >= self.min_range
        valid_depth &= depth <= self.max_range

        finite_xyz = np.isfinite(organized).all(axis=2)
        valid_depth &= finite_xyz

        return points, organized, valid_depth

    def _detect_positive_hazards(self, points: np.ndarray) -> np.ndarray:
        """Keep existing positive obstacle path using plane outliers."""
        sampled = points[:: self.downsample_step]
        plane = self._estimate_ground_plane(sampled)
        if plane is None:
            return np.empty((0, 3), dtype=np.float32)

        normal, offset = plane
        signed_distance = points @ normal + offset

        obstacle_mask = signed_distance > self.positive_height_threshold
        obstacle_mask &= signed_distance < self.positive_max_height

        obstacles = points[obstacle_mask]
        if obstacles.shape[0] < self.min_points_per_cloud:
            return np.empty((0, 3), dtype=np.float32)

        return obstacles

    def _estimate_ground_plane(
        self, points: np.ndarray
    ) -> tuple[np.ndarray, float] | None:
        """Estimate dominant ground plane with lightweight RANSAC."""
        if points.shape[0] < 3:
            return None

        rng = np.random.default_rng()
        best_inlier_count = 0
        best_plane = None

        for _ in range(self.ransac_iterations):
            try:
                sample_ids = rng.choice(points.shape[0], size=3, replace=False)
            except ValueError:
                return None

            p0, p1, p2 = points[sample_ids]
            normal = np.cross(p1 - p0, p2 - p0)
            normal_norm = np.linalg.norm(normal)
            if normal_norm < 1e-6:
                continue

            normal = normal / normal_norm
            offset = -np.dot(normal, p0)
            distance = np.abs(points @ normal + offset)
            inlier_count = int(
                np.count_nonzero(distance < self.plane_distance_threshold)
            )

            if inlier_count > best_inlier_count:
                best_inlier_count = inlier_count
                best_plane = (normal, float(offset))

        return best_plane

    def _detect_negative_hazard_mask(
        self,
        organized_points: np.ndarray,
        valid_depth: np.ndarray,
    ) -> np.ndarray:
        """Detect crater-edge risk from depth discontinuity and slope."""
        depth = organized_points[:, :, 2]
        depth = np.where(valid_depth, depth, np.nan)

        grad_row = np.abs(np.diff(depth, axis=0, prepend=np.nan))
        grad_col = np.abs(np.diff(depth, axis=1, prepend=np.nan))
        gradient = np.nanmax(np.stack((grad_row, grad_col), axis=0), axis=0)

        discontinuity_mask = gradient > self.negative_discontinuity_threshold
        slope_mask = gradient > self.negative_gradient_threshold
        negative_mask = valid_depth & (discontinuity_mask | slope_mask)

        row_limit = int(organized_points.shape[0] * self.negative_min_row_ratio)
        if row_limit > 0:
            negative_mask[:row_limit, :] = False

        neighbor_support = self._neighbor_count(negative_mask) >= 3
        negative_mask &= neighbor_support

        return negative_mask

    @staticmethod
    def _neighbor_count(mask: np.ndarray) -> np.ndarray:
        """Count true neighbors in a 3x3 window."""
        padded = np.pad(mask.astype(np.uint8), 1)
        count = np.zeros_like(mask, dtype=np.uint8)

        for row_shift in range(3):
            for col_shift in range(3):
                if row_shift == 1 and col_shift == 1:
                    continue
                count += padded[
                    row_shift:row_shift + mask.shape[0],
                    col_shift:col_shift + mask.shape[1],
                ]

        return count

    @staticmethod
    def _publish_cloud(
        publisher,
        header,
        points: np.ndarray,
    ) -> None:
        """Publish an xyz PointCloud2 message from Nx3 numpy points."""
        if points.shape[0] == 0:
            cloud_msg = point_cloud2.create_cloud_xyz32(header, [])
        else:
            cloud_msg = point_cloud2.create_cloud_xyz32(
                header,
                points.astype(np.float32).tolist(),
            )
        publisher.publish(cloud_msg)

    def _publish_debug_images(
        self,
        msg: PointCloud2,
        organized_points: np.ndarray,
        valid_depth: np.ndarray,
        negative_mask: np.ndarray,
    ) -> None:
        """Publish mask and edge intensity debug views."""
        depth = np.where(valid_depth, organized_points[:, :, 2], np.nan)
        grad_row = np.abs(np.diff(depth, axis=0, prepend=np.nan))
        grad_col = np.abs(np.diff(depth, axis=1, prepend=np.nan))
        edges = np.nan_to_num(np.maximum(grad_row, grad_col), nan=0.0)

        if np.max(edges) > 0:
            edge_img = (255.0 * edges / np.max(edges)).astype(np.uint8)
        else:
            edge_img = np.zeros_like(edges, dtype=np.uint8)

        mask_img = (negative_mask.astype(np.uint8) * 255).astype(np.uint8)

        if self.mask_publisher is not None:
            self.mask_publisher.publish(self._to_mono8_image(msg, mask_img))
        if self.edges_publisher is not None:
            self.edges_publisher.publish(self._to_mono8_image(msg, edge_img))

    @staticmethod
    def _to_mono8_image(msg: PointCloud2, image: np.ndarray) -> Image:
        """Create a mono8 image message with input point cloud header."""
        image_msg = Image()
        image_msg.header = msg.header
        image_msg.height = image.shape[0]
        image_msg.width = image.shape[1]
        image_msg.encoding = "mono8"
        image_msg.is_bigendian = False
        image_msg.step = image_msg.width
        image_msg.data = image.tobytes()
        return image_msg


def main(args=None):
    """Run the hazard detection node."""
    rclpy.init(args=args)
    node = HazardDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
