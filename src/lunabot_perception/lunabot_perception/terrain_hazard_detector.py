"""Local crater hazard extraction from the front depth camera point cloud."""

from __future__ import annotations

from typing import Iterable

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from lunabot_perception.terrain_hazard_core import GridSpec
from lunabot_perception.terrain_hazard_core import build_elevation_grid
from lunabot_perception.terrain_hazard_core import decay_confidence
from lunabot_perception.terrain_hazard_core import detect_drop_cells
from lunabot_perception.terrain_hazard_core import filter_clusters
from lunabot_perception.terrain_hazard_core import occupancy_values
from lunabot_perception.terrain_hazard_core import pad_hazards
from lunabot_perception.terrain_hazard_core import update_confidence


def quaternion_to_matrix(quaternion: Quaternion) -> np.ndarray:
    """Convert a ROS quaternion into a 3x3 rotation matrix."""
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-6 or not np.isfinite(norm):
        raise ValueError("Received a non-finite transform quaternion.")

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float32,
    )


class TerrainHazardDetector(Node):
    """Estimate local negative obstacles from the front depth point cloud."""

    def __init__(self) -> None:
        """Initialise subscriptions, publishers, and detector state."""
        super().__init__("terrain_hazard_detector")

        self._declare_parameters()

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.points_topic = str(self.get_parameter("hazard_points_topic").value)
        self.grid_topic = str(self.get_parameter("hazard_grid_topic").value)
        self.ready_topic = str(self.get_parameter("hazard_ready_topic").value)
        self.marker_height = float(self.get_parameter("hazard_marker_height").value)
        self.stale_timeout = Duration(
            seconds=float(self.get_parameter("stale_timeout_s").value)
        )
        self.reference_frame_rate = float(
            self.get_parameter("reference_frame_rate").value
        )
        self.log_period = Duration(
            seconds=float(self.get_parameter("log_period_sec").value)
        )
        now_ns = self.get_clock().now().nanoseconds
        self.last_log_time_ns = now_ns
        self.last_confidence_time_ns = now_ns
        self.last_input_time_ns = now_ns

        self.grid = GridSpec(
            min_x=float(self.get_parameter("grid_min_x").value),
            max_x=float(self.get_parameter("grid_max_x").value),
            min_y=float(self.get_parameter("grid_min_y").value),
            max_y=float(self.get_parameter("grid_max_y").value),
            resolution=float(self.get_parameter("grid_resolution").value),
        )

        self.min_points_per_cell = int(
            self.get_parameter("min_points_per_cell").value
        )
        self.neighborhood_radius_cells = int(
            self.get_parameter("neighborhood_radius_cells").value
        )
        self.min_neighbor_cells = int(
            self.get_parameter("min_neighbor_cells").value
        )
        self.drop_threshold = float(self.get_parameter("drop_threshold_m").value)
        self.roughness_threshold = float(
            self.get_parameter("roughness_threshold_m").value
        )
        self.min_cluster_cells = int(
            self.get_parameter("min_cluster_cells").value
        )
        self.hazard_padding_cells = int(
            self.get_parameter("hazard_padding_cells").value
        )
        self.confidence_add = float(self.get_parameter("confidence_add").value)
        self.confidence_decay = float(
            self.get_parameter("confidence_decay").value
        )
        self.confidence_clear = float(
            self.get_parameter("confidence_clear").value
        )
        self.publish_threshold = float(
            self.get_parameter("publish_threshold").value
        )
        self.max_detection_x = float(
            self.get_parameter("max_detection_x").value
        )
        self.edge_margin_cells = int(
            self.get_parameter("edge_margin_cells").value
        )
        self.min_valid_z = float(self.get_parameter("min_valid_z").value)
        self.max_valid_z = float(self.get_parameter("max_valid_z").value)
        self.self_filter_x = float(self.get_parameter("self_filter_x").value)
        self.self_filter_y = float(self.get_parameter("self_filter_y").value)
        self.self_filter_z = float(self.get_parameter("self_filter_z").value)
        self.min_input_points = int(self.get_parameter("min_input_points").value)

        self.confidence = np.zeros(
            (self.grid.height, self.grid.width),
            dtype=np.float32,
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.hazard_points_pub = self.create_publisher(
            PointCloud2,
            self.points_topic,
            10,
        )
        self.hazard_grid_pub = self.create_publisher(
            OccupancyGrid,
            self.grid_topic,
            10,
        )
        self.hazard_ready_pub = self.create_publisher(
            Bool,
            self.ready_topic,
            10,
        )
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.point_cloud_callback,
            qos_profile_sensor_data,
        )
        self.stale_timer = self.create_timer(0.5, self.handle_stale_input)

    def _declare_parameters(self) -> None:
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("input_topic", "/camera_front/points")
        self.declare_parameter("hazard_points_topic", "/terrain_hazard/points")
        self.declare_parameter("hazard_grid_topic", "/terrain_hazard/grid")
        self.declare_parameter("hazard_ready_topic", "/terrain_hazard/ready")
        self.declare_parameter("grid_min_x", 0.40)
        self.declare_parameter("grid_max_x", 2.40)
        self.declare_parameter("grid_min_y", -1.00)
        self.declare_parameter("grid_max_y", 1.00)
        self.declare_parameter("grid_resolution", 0.10)
        self.declare_parameter("min_points_per_cell", 3)
        self.declare_parameter("neighborhood_radius_cells", 2)
        self.declare_parameter("min_neighbor_cells", 6)
        self.declare_parameter("drop_threshold_m", 0.14)
        self.declare_parameter("roughness_threshold_m", 0.10)
        self.declare_parameter("min_cluster_cells", 4)
        self.declare_parameter("hazard_padding_cells", 1)
        self.declare_parameter("confidence_add", 0.45)
        self.declare_parameter("confidence_decay", 0.82)
        self.declare_parameter("confidence_clear", 0.20)
        self.declare_parameter("publish_threshold", 0.90)
        self.declare_parameter("reference_frame_rate", 5.0)
        self.declare_parameter("max_detection_x", 2.10)
        self.declare_parameter("edge_margin_cells", 1)
        self.declare_parameter("min_valid_z", -1.50)
        self.declare_parameter("max_valid_z", 1.00)
        self.declare_parameter("self_filter_x", 0.70)
        self.declare_parameter("self_filter_y", 0.40)
        self.declare_parameter("self_filter_z", -0.30)
        self.declare_parameter("hazard_marker_height", 0.20)
        self.declare_parameter("stale_timeout_s", 1.0)
        self.declare_parameter("log_period_sec", 5.0)
        self.declare_parameter("min_input_points", 50)

    def point_cloud_callback(self, msg: PointCloud2) -> None:
        """Convert a front point cloud into a hazard grid."""
        now = self.get_clock().now()
        self.last_input_time_ns = now.nanoseconds

        points = self.decode_points(msg)
        if points is None or points.shape[0] < self.min_input_points:
            self.publish_unknown(msg.header, now, sensor_ready=False)
            return

        transformed = self.transform_points(points, msg.header)
        if transformed is None or transformed.shape[0] < self.min_input_points:
            self.publish_unknown(msg.header, now, sensor_ready=False)
            return

        roi_points = self.crop_points(transformed)
        if roi_points.shape[0] < self.min_input_points:
            self.publish_unknown(msg.header, now, sensor_ready=False)
            return

        min_height, max_height, counts = build_elevation_grid(roi_points, self.grid)
        evidence, unknown = detect_drop_cells(
            min_height=min_height,
            max_height=max_height,
            counts=counts,
            min_points_per_cell=self.min_points_per_cell,
            neighborhood_radius_cells=self.neighborhood_radius_cells,
            min_neighbor_cells=self.min_neighbor_cells,
            drop_threshold=self.drop_threshold,
            roughness_threshold=self.roughness_threshold,
            edge_margin_cells=self.edge_margin_cells,
            max_detection_x=self.max_detection_x,
            grid=self.grid,
        )
        evidence = filter_clusters(evidence, self.min_cluster_cells)
        evidence = pad_hazards(evidence, self.hazard_padding_cells)
        supported = counts >= self.min_points_per_cell
        scale = self.confidence_scale(now)
        self.confidence, hazards = update_confidence(
            self.confidence,
            evidence=evidence,
            supported=supported,
            confidence_decay=self.confidence_decay,
            confidence_add=self.confidence_add,
            confidence_clear=self.confidence_clear,
            publish_threshold=self.publish_threshold,
            scale=scale,
        )
        unknown |= ~supported
        unknown &= ~hazards
        self.publish_outputs(msg.header, hazards, unknown, sensor_ready=True)

        if (
            self._elapsed_ns(now.nanoseconds, self.last_log_time_ns)
            >= self.log_period.nanoseconds
        ):
            self.get_logger().info(
                "Published %d crater hazard cells from %d front camera points."
                % (int(hazards.sum()), roi_points.shape[0])
            )
            self.last_log_time_ns = now.nanoseconds

    def decode_points(self, msg: PointCloud2) -> np.ndarray | None:
        """Decode x, y, z points from a PointCloud2 message."""
        field_names = {field.name for field in msg.fields}
        if not {"x", "y", "z"}.issubset(field_names):
            self.get_logger().error(
                "Front camera point cloud is missing x/y/z fields."
            )
            return None

        raw_points = point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )

        try:
            if isinstance(raw_points, np.ndarray):
                if raw_points.dtype.names is None:
                    points = np.asarray(raw_points, dtype=np.float32)
                else:
                    points = np.column_stack(
                        [raw_points["x"], raw_points["y"], raw_points["z"]]
                    ).astype(np.float32, copy=False)
            else:
                points = np.asarray(list(raw_points), dtype=np.float32)
        except (IndexError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(
                f"Failed to decode front camera point cloud layout: {exc}"
            )
            return None

        if points.ndim != 2 or points.shape[1] != 3:
            self.get_logger().error(
                "Front camera point cloud did not decode into Nx3 data."
            )
            return None

        finite_mask = np.all(np.isfinite(points), axis=1)
        return points[finite_mask]

    def transform_points(
        self,
        points: np.ndarray,
        header: Header,
    ) -> np.ndarray | None:
        """Transform the point cloud into the rover base frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                header.frame_id,
                Time.from_msg(header.stamp),
                timeout=Duration(seconds=0.1),
            )
            rotation = quaternion_to_matrix(transform.transform.rotation)
        except (TransformException, ValueError) as exc:
            self.get_logger().warn(
                f"Failed to transform terrain points into {self.base_frame}: {exc}"
            )
            return None

        translation = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float32,
        )
        if not np.all(np.isfinite(translation)):
            self.get_logger().warn("Received a non-finite transform translation.")
            return None

        transformed = (points @ rotation.T) + translation
        finite_mask = np.all(np.isfinite(transformed), axis=1)
        return transformed[finite_mask]

    def crop_points(self, points: np.ndarray) -> np.ndarray:
        """Keep only plausible terrain points inside the local grid."""
        mask = (
            (points[:, 0] >= self.grid.min_x)
            & (points[:, 0] <= self.grid.max_x)
            & (points[:, 1] >= self.grid.min_y)
            & (points[:, 1] <= self.grid.max_y)
            & (points[:, 0] <= self.max_detection_x)
            & (points[:, 2] >= self.min_valid_z)
            & (points[:, 2] <= self.max_valid_z)
        )
        mask &= ~(
            (points[:, 0] <= self.self_filter_x)
            & (np.abs(points[:, 1]) <= self.self_filter_y)
            & (points[:, 2] >= self.self_filter_z)
        )
        return points[mask]

    def confidence_scale(self, now: Time) -> float:
        """Scale confidence updates by elapsed time rather than callback count."""
        elapsed_ns = self._elapsed_ns(now.nanoseconds, self.last_confidence_time_ns)
        elapsed = max(elapsed_ns / 1e9, 0.0)
        self.last_confidence_time_ns = now.nanoseconds
        return max(elapsed * self.reference_frame_rate, 0.25)

    def handle_stale_input(self) -> None:
        """Publish an all-unknown grid when the depth stream goes stale."""
        now = self.get_clock().now()
        if now.nanoseconds <= 0:
            return
        if (
            self._elapsed_ns(now.nanoseconds, self.last_input_time_ns)
            < self.stale_timeout.nanoseconds
        ):
            return

        self.publish_unknown(self.make_header(now), now, sensor_ready=False)

    def publish_unknown(self, header: Header, now: Time, *, sensor_ready: bool) -> None:
        """Publish a conservative unknown grid when terrain support is absent."""
        scale = self.confidence_scale(now)
        self.confidence, _ = decay_confidence(
            self.confidence,
            confidence_decay=self.confidence_decay,
            publish_threshold=self.publish_threshold,
            scale=scale,
        )
        hazards = np.zeros((self.grid.height, self.grid.width), dtype=bool)
        unknown = np.ones((self.grid.height, self.grid.width), dtype=bool)
        self.publish_outputs(header, hazards, unknown, sensor_ready=sensor_ready)

    def publish_outputs(
        self,
        input_header: Header,
        hazards: np.ndarray,
        unknown: np.ndarray,
        *,
        sensor_ready: bool,
    ) -> None:
        """Publish terrain hazards as points and a tri-state occupancy grid."""
        header = Header()
        header.stamp = input_header.stamp
        header.frame_id = self.base_frame

        hazard_points = [
            [
                self.grid.min_x + (x_idx + 0.5) * self.grid.resolution,
                self.grid.min_y + (y_idx + 0.5) * self.grid.resolution,
                self.marker_height,
            ]
            for y_idx, x_idx in np.argwhere(hazards)
        ]
        self.hazard_points_pub.publish(
            point_cloud2.create_cloud_xyz32(header, hazard_points)
        )

        grid_msg = OccupancyGrid()
        grid_msg.header = header
        grid_msg.info.map_load_time = TimeMsg()
        grid_msg.info.resolution = self.grid.resolution
        grid_msg.info.width = self.grid.width
        grid_msg.info.height = self.grid.height
        grid_msg.info.origin.position.x = self.grid.min_x
        grid_msg.info.origin.position.y = self.grid.min_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = occupancy_values(
            hazards=hazards,
            unknown=unknown,
        ).reshape(-1).tolist()
        self.hazard_grid_pub.publish(grid_msg)
        self.hazard_ready_pub.publish(Bool(data=sensor_ready))

    def make_header(self, now: Time) -> Header:
        """Build a base-frame header using the current ROS clock."""
        header = Header()
        header.stamp = now.to_msg()
        header.frame_id = self.base_frame
        return header

    @staticmethod
    def _elapsed_ns(now_ns: int, then_ns: int) -> int:
        """Return a safe elapsed time that tolerates sim time starting late."""
        if now_ns <= 0 or then_ns <= 0 or now_ns < then_ns:
            return 0
        return now_ns - then_ns


def main(args: Iterable[str] | None = None) -> None:
    """Run the terrain hazard detector node."""
    rclpy.init(args=args)
    node = TerrainHazardDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # pragma: no cover - shutdown can already be in progress
            pass


if __name__ == "__main__":
    main()
