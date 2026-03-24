"""Local crater hazard extraction from the front depth camera point cloud."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
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
from tf2_ros import Buffer, TransformException, TransformListener

INPUT_TOPIC = "/camera_front/points"
BASE_FRAME = "base_link"
HAZARD_POINTS_TOPIC = "/crater_hazards/points"
HAZARD_GRID_TOPIC = "/crater_hazards/grid"


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


@dataclass(frozen=True)
class GridSpec:
    """Local grid definition for crater hazard extraction."""

    min_x: float
    max_x: float
    min_y: float
    max_y: float
    resolution: float

    @property
    def width(self) -> int:
        """Return grid width in cells."""
        return int(round((self.max_x - self.min_x) / self.resolution))

    @property
    def height(self) -> int:
        """Return grid height in cells."""
        return int(round((self.max_y - self.min_y) / self.resolution))


class HazardDetectionNode(Node):
    """Estimate local negative obstacles from depth."""

    def __init__(self) -> None:
        """Initialise subscriptions, publishers, and detector state."""
        super().__init__("crater_hazard_detector")

        self.declare_parameter("log_period_sec", 5.0)

        self.base_frame = BASE_FRAME
        self.grid = GridSpec(
            min_x=0.40,
            max_x=2.40,
            min_y=-1.00,
            max_y=1.00,
            resolution=0.10,
        )

        self.min_points_per_cell = 3
        self.neighborhood_radius_cells = 2
        self.min_neighbor_cells = 6
        self.drop_threshold = 0.14
        self.roughness_threshold = 0.10
        self.min_cluster_cells = 4
        self.hazard_padding_cells = 1
        self.confidence_add = 0.45
        self.confidence_decay = 0.82
        self.confidence_clear = 0.20
        self.publish_threshold = 0.90
        self.reference_frame_rate = 5.0
        self.max_detection_x = 2.10
        self.edge_margin_cells = 1
        self.min_valid_z = -1.50
        self.max_valid_z = 1.00
        self.self_filter_x = 0.70
        self.self_filter_y = 0.40
        self.self_filter_z = -0.30
        self.hazard_marker_height = 0.20
        self.log_period = Duration(
            seconds=float(self.get_parameter("log_period_sec").value)
        )
        self.last_log_time = self.get_clock().now() - self.log_period
        self.last_confidence_time = self.get_clock().now()
        self.confidence = np.zeros(
            (self.grid.height, self.grid.width),
            dtype=np.float32,
        )

        assert self.grid.max_x > self.grid.min_x
        assert self.grid.max_y > self.grid.min_y
        assert self.grid.resolution > 0.0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.hazard_points_pub = self.create_publisher(
            PointCloud2,
            HAZARD_POINTS_TOPIC,
            10,
        )
        self.hazard_grid_pub = self.create_publisher(
            OccupancyGrid,
            HAZARD_GRID_TOPIC,
            10,
        )
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            INPUT_TOPIC,
            self.point_cloud_callback,
            qos_profile_sensor_data,
        )

    def point_cloud_callback(self, msg: PointCloud2) -> None:
        """Convert a front point cloud into a hazard grid."""
        now = self.get_clock().now()
        points = self.decode_points(msg)
        if points is None or points.size == 0:
            self.publish_outputs(msg.header, self.decay_confidence(now))
            return

        transformed = self.transform_points(points, msg.header)
        if transformed is None or transformed.size == 0:
            self.publish_outputs(msg.header, self.decay_confidence(now))
            return

        roi_points = self.crop_points(transformed)
        if roi_points.shape[0] < 50:
            self.publish_outputs(msg.header, self.decay_confidence(now))
            return

        min_height, max_height, counts = self.build_elevation_grid(roi_points)
        evidence = self.detect_drop_cells(min_height, max_height, counts)
        evidence = self.filter_clusters(evidence)
        evidence = self.pad_hazards(evidence)
        hazards = self.update_confidence(evidence, counts, now)
        self.publish_outputs(msg.header, hazards)

        if now - self.last_log_time >= self.log_period:
            self.get_logger().info(
                "Published %d crater hazard cells from %d front camera points."
                % (int(hazards.sum()), roi_points.shape[0])
            )
            self.last_log_time = now

    def decode_points(self, msg: PointCloud2) -> np.ndarray | None:
        """Decode x/y/z points from the front camera cloud."""
        field_names = {field.name for field in msg.fields}
        if not {"x", "y", "z"}.issubset(field_names):
            self.get_logger().error(
                "Front camera point cloud is missing x/y/z fields."
            )
            return None

        try:
            raw_points = point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        except (TypeError, ValueError) as exc:
            self.get_logger().error(
                f"Failed to decode front camera point cloud: {exc}"
            )
            return None

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
                f"Failed to normalize front camera point cloud layout: {exc}"
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
                f"Failed to transform crater points into {self.base_frame}: {exc}"
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

    def build_elevation_grid(
        self, points: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Build per-cell terrain height statistics."""
        height = self.grid.height
        width = self.grid.width
        min_height = np.full((height, width), np.inf, dtype=np.float32)
        max_height = np.full((height, width), -np.inf, dtype=np.float32)
        counts = np.zeros((height, width), dtype=np.int32)

        xs = points[:, 0]
        ys = points[:, 1]
        zs = points[:, 2]

        x_indices = np.floor(
            (xs - self.grid.min_x) / self.grid.resolution
        ).astype(int)
        y_indices = np.floor(
            (ys - self.grid.min_y) / self.grid.resolution
        ).astype(int)
        valid = (
            (x_indices >= 0)
            & (x_indices < width)
            & (y_indices >= 0)
            & (y_indices < height)
        )

        for x_idx, y_idx, z_val in zip(
            x_indices[valid], y_indices[valid], zs[valid], strict=False
        ):
            counts[y_idx, x_idx] += 1
            if z_val < min_height[y_idx, x_idx]:
                min_height[y_idx, x_idx] = z_val
            if z_val > max_height[y_idx, x_idx]:
                max_height[y_idx, x_idx] = z_val

        return min_height, max_height, counts

    def detect_drop_cells(
        self,
        min_height: np.ndarray,
        max_height: np.ndarray,
        counts: np.ndarray,
    ) -> np.ndarray:
        """Detect cells below a locally smooth terrain neighborhood."""
        hazards = np.zeros((self.grid.height, self.grid.width), dtype=bool)
        supported = counts >= self.min_points_per_cell

        for y_idx in range(
            self.edge_margin_cells,
            self.grid.height - self.edge_margin_cells,
        ):
            for x_idx in range(
                self.edge_margin_cells, self.grid.width - self.edge_margin_cells
            ):
                if not supported[y_idx, x_idx]:
                    continue

                x_center = self.grid.min_x + (
                    (x_idx + 0.5) * self.grid.resolution
                )
                if x_center > self.max_detection_x:
                    continue

                min_y = max(0, y_idx - self.neighborhood_radius_cells)
                max_y = min(
                    self.grid.height,
                    y_idx + self.neighborhood_radius_cells + 1,
                )
                min_x = max(0, x_idx - self.neighborhood_radius_cells)
                max_x = min(
                    self.grid.width,
                    x_idx + self.neighborhood_radius_cells + 1,
                )
                neighborhood = supported[min_y:max_y, min_x:max_x]
                neighbor_min = min_height[min_y:max_y, min_x:max_x]

                neighbor_values = neighbor_min[neighborhood]
                if neighbor_values.size < self.min_neighbor_cells:
                    continue

                reference_height = float(np.median(neighbor_values))
                roughness = float(
                    np.percentile(neighbor_values, 90)
                    - np.percentile(neighbor_values, 10)
                )
                local_span = (
                    max_height[y_idx, x_idx] - min_height[y_idx, x_idx]
                )
                if (
                    roughness > self.roughness_threshold
                    or local_span > 0.18
                ):
                    continue

                if (
                    min_height[y_idx, x_idx]
                    <= reference_height - self.drop_threshold
                ):
                    hazards[y_idx, x_idx] = True

        return hazards

    def filter_clusters(self, hazards: np.ndarray) -> np.ndarray:
        """Remove tiny hazard clusters."""
        if not hazards.any():
            return hazards

        filtered = np.zeros_like(hazards)
        visited = np.zeros_like(hazards, dtype=bool)
        offsets = ((1, 0), (-1, 0), (0, 1), (0, -1))

        for start_y, start_x in np.argwhere(hazards):
            if visited[start_y, start_x]:
                continue

            queue: deque[tuple[int, int]] = deque([(start_y, start_x)])
            cluster: list[tuple[int, int]] = []
            visited[start_y, start_x] = True

            while queue:
                y_idx, x_idx = queue.popleft()
                cluster.append((y_idx, x_idx))
                for dy, dx in offsets:
                    next_y = y_idx + dy
                    next_x = x_idx + dx
                    if (
                        next_y < 0
                        or next_y >= hazards.shape[0]
                        or next_x < 0
                        or next_x >= hazards.shape[1]
                        or visited[next_y, next_x]
                        or not hazards[next_y, next_x]
                    ):
                        continue
                    visited[next_y, next_x] = True
                    queue.append((next_y, next_x))

            if len(cluster) >= self.min_cluster_cells:
                for y_idx, x_idx in cluster:
                    filtered[y_idx, x_idx] = True

        return filtered

    def pad_hazards(self, hazards: np.ndarray) -> np.ndarray:
        """Dilate hazard cells slightly around crater detections."""
        if self.hazard_padding_cells <= 0 or not hazards.any():
            return hazards

        padded = hazards.copy()
        for y_idx, x_idx in np.argwhere(hazards):
            min_y = max(0, y_idx - self.hazard_padding_cells)
            max_y = min(
                hazards.shape[0],
                y_idx + self.hazard_padding_cells + 1,
            )
            min_x = max(0, x_idx - self.hazard_padding_cells)
            max_x = min(
                hazards.shape[1],
                x_idx + self.hazard_padding_cells + 1,
            )
            padded[min_y:max_y, min_x:max_x] = True
        return padded

    def update_confidence(
        self,
        evidence: np.ndarray,
        counts: np.ndarray,
        now: Time,
    ) -> np.ndarray:
        """Fuse hazard evidence over time."""
        scale = self.confidence_scale(now)
        supported = counts >= self.min_points_per_cell
        self.confidence *= self.confidence_decay**scale
        self.confidence[evidence] += self.confidence_add * scale
        self.confidence[supported & ~evidence] -= self.confidence_clear * scale
        self.confidence = np.clip(self.confidence, 0.0, 2.0)
        return self.confidence >= self.publish_threshold

    def decay_confidence(self, now: Time) -> np.ndarray:
        """Decay confidence when no fresh terrain evidence is available."""
        scale = self.confidence_scale(now)
        self.confidence *= self.confidence_decay**scale
        self.confidence = np.clip(self.confidence, 0.0, 2.0)
        return self.confidence >= self.publish_threshold

    def confidence_scale(self, now: Time) -> float:
        """Scale confidence updates by elapsed time instead of raw callback count."""
        elapsed = max(
            (now - self.last_confidence_time).nanoseconds / 1e9,
            0.0,
        )
        self.last_confidence_time = now
        return max(elapsed * self.reference_frame_rate, 0.25)

    def publish_outputs(self, input_header: Header, hazards: np.ndarray) -> None:
        """Publish crater hazards as a point cloud and occupancy grid."""
        header = Header()
        header.stamp = input_header.stamp
        header.frame_id = self.base_frame

        hazard_points = [
            [
                self.grid.min_x + (x_idx + 0.5) * self.grid.resolution,
                self.grid.min_y + (y_idx + 0.5) * self.grid.resolution,
                self.hazard_marker_height,
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
        grid_msg.data = [
            100 if occupied else 0
            for occupied in hazards.reshape(-1)
        ]
        self.hazard_grid_pub.publish(grid_msg)


def main(args: Iterable[str] | None = None) -> None:
    """Run the crater hazard extraction node."""
    rclpy.init(args=args)
    node = HazardDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
