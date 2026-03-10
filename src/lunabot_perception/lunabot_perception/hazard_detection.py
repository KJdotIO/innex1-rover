"""Hazard detection node for processing depth camera point clouds."""

from collections import deque

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32


class HazardDetectionNode(Node):
    """Detect positive and negative terrain hazards from depth point clouds."""

    def __init__(self):
        """Initialize node, parameters and publishers."""
        super().__init__("hazard_detection_node")

        self._declare_parameters()

        self.hazard_pub = self.create_publisher(
            PointCloud2,
            "/hazards/front",
            10,
        )
        self.positive_pub = self.create_publisher(
            PointCloud2, "/hazards/positive", 10
        )
        self.crater_pub = self.create_publisher(
            PointCloud2, "/hazards/crater", 10
        )
        self.confidence_pub = self.create_publisher(
            Float32, "/hazards/confidence", 10
        )

        self.subscription = self.create_subscription(
            PointCloud2, "/camera_front/points", self.listener_callback, 10
        )

        self._history = deque(
            maxlen=int(self.get_parameter("history_frames").value)
        )

        self.get_logger().info(
            "Hazard detection initialised with Open3D pipeline"
        )

    def _declare_parameters(self):
        self.declare_parameter("voxel_size", 0.05)
        self.declare_parameter("roi_x_min", 0.15)
        self.declare_parameter("roi_x_max", 3.0)
        self.declare_parameter("roi_y_abs_max", 1.8)
        self.declare_parameter("roi_z_min", -0.80)
        self.declare_parameter("roi_z_max", 0.80)

        self.declare_parameter("sor_nb_neighbors", 20)
        self.declare_parameter("sor_std_ratio", 2.0)
        self.declare_parameter("use_radius_outlier", False)
        self.declare_parameter("radius_nb_points", 12)
        self.declare_parameter("radius_m", 0.08)

        self.declare_parameter("plane_distance_threshold", 0.035)
        self.declare_parameter("plane_ransac_n", 3)
        self.declare_parameter("plane_num_iterations", 150)

        self.declare_parameter("positive_height_m", 0.08)
        self.declare_parameter("negative_depth_m", 0.06)
        self.declare_parameter("crater_edge_depth_m", 0.03)
        self.declare_parameter("steep_normal_cos_threshold", 0.65)

        self.declare_parameter("history_frames", 3)
        self.declare_parameter("history_voxel_size", 0.04)
        self.declare_parameter("min_hazard_points", 20)
        self.declare_parameter("confidence_k", 6.0)

    def listener_callback(self, msg):
        """Process incoming depth cloud and publish hazards."""
        points = self._extract_points(msg)
        if points.size == 0:
            self._publish_empty(msg.header)
            return

        points = self._crop_roi(points)
        if points.size == 0:
            self._publish_empty(msg.header)
            return

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        pcd = pcd.voxel_down_sample(
            float(self.get_parameter("voxel_size").value)
        )
        if len(pcd.points) == 0:
            self._publish_empty(msg.header)
            return

        pcd, _ = pcd.remove_statistical_outlier(
            nb_neighbors=int(self.get_parameter("sor_nb_neighbors").value),
            std_ratio=float(self.get_parameter("sor_std_ratio").value),
        )

        if bool(self.get_parameter("use_radius_outlier").value):
            pcd, _ = pcd.remove_radius_outlier(
                nb_points=int(self.get_parameter("radius_nb_points").value),
                radius=float(self.get_parameter("radius_m").value),
            )

        if len(pcd.points) < 30:
            self._publish_empty(msg.header)
            return

        all_points = np.asarray(pcd.points)

        plane_model, _ = pcd.segment_plane(
            distance_threshold=float(
                self.get_parameter("plane_distance_threshold").value
            ),
            ransac_n=int(self.get_parameter("plane_ransac_n").value),
            num_iterations=int(
                self.get_parameter("plane_num_iterations").value
            ),
        )
        plane_normal = np.array(plane_model[:3], dtype=np.float64)
        normal_norm = np.linalg.norm(plane_normal)
        if normal_norm < 1e-6:
            self._publish_empty(msg.header)
            return
        plane_normal = plane_normal / normal_norm

        signed_dist = (
            all_points @ plane_normal + float(plane_model[3])
        )

        positive_idx = np.where(
            signed_dist > float(self.get_parameter("positive_height_m").value)
        )[0]

        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.12,
                max_nn=30,
            )
        )
        normals = np.asarray(pcd.normals)
        normal_alignment = np.abs(normals @ plane_normal)

        negative_idx = np.where(
            signed_dist < -float(self.get_parameter("negative_depth_m").value)
        )[0]
        crater_edge_depth = float(
            self.get_parameter("crater_edge_depth_m").value
        )
        steep_normal_cos = float(
            self.get_parameter("steep_normal_cos_threshold").value
        )
        steep_negative_idx = np.where(
            (signed_dist < -crater_edge_depth)
            & (normal_alignment < steep_normal_cos)
        )[0]

        crater_idx = np.union1d(negative_idx, steep_negative_idx)
        hazard_idx = np.union1d(positive_idx, crater_idx)

        min_hazard_points = int(self.get_parameter("min_hazard_points").value)
        if hazard_idx.size < min_hazard_points:
            self._history.clear()
            self._publish_empty(msg.header)
            return

        hazards = all_points[hazard_idx]
        positives = all_points[positive_idx]
        craters = all_points[crater_idx]

        self._history.append(hazards)
        smoothed_hazards = self._aggregate_history()

        confidence = self._hazard_confidence(
            total_points=len(all_points),
            positive_points=len(positives),
            crater_points=len(craters),
        )

        self.hazard_pub.publish(
            point_cloud2.create_cloud_xyz32(msg.header, smoothed_hazards)
        )
        positive_msg = point_cloud2.create_cloud_xyz32(msg.header, positives)
        crater_msg = point_cloud2.create_cloud_xyz32(msg.header, craters)
        self.positive_pub.publish(positive_msg)
        self.crater_pub.publish(crater_msg)
        self.confidence_pub.publish(Float32(data=float(confidence)))

    def _extract_points(self, msg):
        generator = point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )
        rows = list(generator)
        if not rows:
            return np.empty((0, 3), dtype=np.float64)

        first = rows[0]
        if isinstance(first, np.void) and getattr(first, "dtype", None):
            if first.dtype.names and all(
                name in first.dtype.names for name in ("x", "y", "z")
            ):
                x_vals = np.asarray(
                    [row["x"] for row in rows], dtype=np.float64
                )
                y_vals = np.asarray(
                    [row["y"] for row in rows], dtype=np.float64
                )
                z_vals = np.asarray(
                    [row["z"] for row in rows], dtype=np.float64
                )
                points = np.column_stack([x_vals, y_vals, z_vals])
            else:
                points = np.asarray(
                    [[row[0], row[1], row[2]] for row in rows],
                    dtype=np.float64,
                )
        else:
            points = np.asarray(
                [[row[0], row[1], row[2]] for row in rows],
                dtype=np.float64,
            )

        finite_mask = np.all(np.isfinite(points), axis=1)
        return points[finite_mask]

    def _crop_roi(self, points):
        x_min = float(self.get_parameter("roi_x_min").value)
        x_max = float(self.get_parameter("roi_x_max").value)
        y_abs_max = float(self.get_parameter("roi_y_abs_max").value)
        z_min = float(self.get_parameter("roi_z_min").value)
        z_max = float(self.get_parameter("roi_z_max").value)

        mask = (
            (points[:, 0] >= x_min)
            & (points[:, 0] <= x_max)
            & (np.abs(points[:, 1]) <= y_abs_max)
            & (points[:, 2] >= z_min)
            & (points[:, 2] <= z_max)
        )
        return points[mask]

    def _aggregate_history(self):
        if not self._history:
            return np.empty((0, 3), dtype=np.float64)

        stacked = np.vstack(self._history)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(stacked)
        pcd = pcd.voxel_down_sample(
            float(self.get_parameter("history_voxel_size").value)
        )
        return np.asarray(pcd.points)

    def _hazard_confidence(self, total_points, positive_points, crater_points):
        if total_points <= 0:
            return 0.0

        hazard_fraction = (positive_points + crater_points) / float(
            total_points
        )
        crater_fraction = crater_points / float(total_points)
        k = float(self.get_parameter("confidence_k").value)

        weighted_fraction = 0.7 * hazard_fraction + 0.3 * crater_fraction
        score = 1.0 - np.exp(-k * weighted_fraction)
        return float(np.clip(score, 0.0, 1.0))

    def _publish_empty(self, header):
        empty = np.empty((0, 3), dtype=np.float32)
        msg = point_cloud2.create_cloud_xyz32(header, empty)
        self.hazard_pub.publish(msg)
        self.positive_pub.publish(msg)
        self.crater_pub.publish(msg)
        self.confidence_pub.publish(Float32(data=0.0))


def main(args=None):
    """Run the hazard detection node."""
    rclpy.init(args=args)
    node = HazardDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
