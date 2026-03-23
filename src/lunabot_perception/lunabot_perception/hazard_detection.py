"""Crater hazard extraction from the front depth camera point cloud."""

from __future__ import annotations

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
    """Estimate a local crater hazard map from the front RGB-D point cloud."""

    def __init__(self) -> None:
        """Initialise subscriptions, publishers, and tuning parameters."""
        super().__init__("crater_hazard_detector")

        self.declare_parameter("drop_threshold", 0.10)
        self.declare_parameter("min_points_per_cell", 2)
        self.declare_parameter("hazard_padding_cells", 1)
        self.declare_parameter("log_period_sec", 5.0)

        self.base_frame = BASE_FRAME
        self.grid = GridSpec(
            min_x=0.35,
            max_x=2.50,
            min_y=-1.20,
            max_y=1.20,
            resolution=0.10,
        )
        self.fit_max_x = 2.00
        self.fit_lateral_limit = 1.00
        self.ground_inlier_margin = 0.04
        self.ground_above_margin = 0.10
        self.drop_threshold = float(self.get_parameter("drop_threshold").value)
        self.min_points_per_cell = int(self.get_parameter("min_points_per_cell").value)
        self.hazard_padding_cells = int(
            self.get_parameter("hazard_padding_cells").value
        )
        self.hazard_marker_height = 0.25
        self.log_period = Duration(
            seconds=float(self.get_parameter("log_period_sec").value)
        )
        self.last_log_time = self.get_clock().now() - self.log_period

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        assert self.grid.max_x > self.grid.min_x
        assert self.grid.max_y > self.grid.min_y
        assert self.grid.resolution > 0.0
        assert self.drop_threshold > 0.0
        assert self.min_points_per_cell > 0
        assert self.hazard_padding_cells >= 0

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
        """Convert a front point cloud into hazard points and a debug grid."""
        field_names = {field.name for field in msg.fields}
        if not {"x", "y", "z"}.issubset(field_names):
            self.get_logger().error("Front camera point cloud is missing x/y/z fields.")
            self.publish_outputs(
                msg.header,
                np.zeros((0, 3), dtype=np.float32),
                np.zeros((self.grid.height, self.grid.width), dtype=bool),
            )
            return

        points = np.array(
            list(
                point_cloud2.read_points(
                    msg,
                    field_names=("x", "y", "z"),
                    skip_nans=True,
                )
            ),
            dtype=np.float32,
        )
        empty_cells = np.zeros((self.grid.height, self.grid.width), dtype=bool)
        if points.size == 0:
            self.publish_outputs(
                msg.header,
                np.zeros((0, 3), dtype=np.float32),
                empty_cells,
            )
            return

        transformed = self.transform_points(points, msg.header)
        if transformed is None or transformed.size == 0:
            self.publish_outputs(
                msg.header,
                np.zeros((0, 3), dtype=np.float32),
                empty_cells,
            )
            return

        roi_mask = (
            (transformed[:, 0] >= self.grid.min_x)
            & (transformed[:, 0] <= self.grid.max_x)
            & (transformed[:, 1] >= self.grid.min_y)
            & (transformed[:, 1] <= self.grid.max_y)
            & np.isfinite(transformed[:, 2])
        )
        roi_points = transformed[roi_mask]
        if roi_points.shape[0] < 50:
            self.publish_outputs(
                msg.header,
                np.zeros((0, 3), dtype=np.float32),
                empty_cells,
            )
            return

        plane = self.fit_ground_plane(roi_points)
        if plane is None:
            self.publish_outputs(
                msg.header,
                np.zeros((0, 3), dtype=np.float32),
                empty_cells,
            )
            return

        hazard_cells, hazard_points = self.extract_hazards(roi_points, plane)
        self.publish_outputs(msg.header, hazard_points, hazard_cells)

        now = self.get_clock().now()
        if now - self.last_log_time >= self.log_period:
            self.get_logger().info(
                "Published %d crater hazard cells from %d front camera points."
                % (
                    int(hazard_cells.sum()) if hazard_cells is not None else 0,
                    roi_points.shape[0],
                )
            )
            self.last_log_time = now

    def transform_points(self, points: np.ndarray, header: Header) -> np.ndarray | None:
        """Transform the point cloud into the rover base frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                header.frame_id,
                Time.from_msg(header.stamp),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Failed to transform crater points into {self.base_frame}: {exc}"
            )
            return None

        rotation = quaternion_to_matrix(transform.transform.rotation)
        translation = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float32,
        )
        return (points @ rotation.T) + translation

    def fit_ground_plane(self, points: np.ndarray) -> np.ndarray | None:
        """Fit a simple ground plane z=ax+by+c over near-field points."""
        plane_mask = (
            (points[:, 0] >= self.grid.min_x)
            & (points[:, 0] <= self.fit_max_x)
            & (np.abs(points[:, 1]) <= self.fit_lateral_limit)
        )
        candidates = points[plane_mask]
        if candidates.shape[0] < 50:
            return None

        a_matrix = np.c_[
            candidates[:, 0],
            candidates[:, 1],
            np.ones(candidates.shape[0]),
        ]
        z_values = candidates[:, 2]
        coeffs, *_ = np.linalg.lstsq(a_matrix, z_values, rcond=None)

        for _ in range(2):
            plane_heights = a_matrix @ coeffs
            residuals = z_values - plane_heights
            inliers = (residuals >= -self.ground_inlier_margin) & (
                residuals <= self.ground_above_margin
            )
            if inliers.sum() < 30:
                break
            coeffs, *_ = np.linalg.lstsq(
                a_matrix[inliers],
                z_values[inliers],
                rcond=None,
            )

        return coeffs.astype(np.float32)

    def extract_hazards(
        self, points: np.ndarray, plane: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Build a local hazard grid from depth below the fitted plane."""
        width = self.grid.width
        height = self.grid.height

        min_residual = np.full((height, width), np.inf, dtype=np.float32)
        counts = np.zeros((height, width), dtype=np.int32)

        xs = points[:, 0]
        ys = points[:, 1]
        plane_height = plane[0] * xs + plane[1] * ys + plane[2]
        residuals = points[:, 2] - plane_height

        x_indices = np.floor((xs - self.grid.min_x) / self.grid.resolution).astype(int)
        y_indices = np.floor((ys - self.grid.min_y) / self.grid.resolution).astype(int)
        valid = (
            (x_indices >= 0)
            & (x_indices < width)
            & (y_indices >= 0)
            & (y_indices < height)
        )

        for x_idx, y_idx, residual in zip(
            x_indices[valid], y_indices[valid], residuals[valid], strict=False
        ):
            counts[y_idx, x_idx] += 1
            if residual < min_residual[y_idx, x_idx]:
                min_residual[y_idx, x_idx] = residual

        hazards = (counts >= self.min_points_per_cell) & (
            min_residual <= -self.drop_threshold
        )
        hazards = self.pad_hazards(hazards)

        hazard_points: list[list[float]] = []
        for y_idx, x_idx in np.argwhere(hazards):
            hazard_points.append(
                [
                    self.grid.min_x + (x_idx + 0.5) * self.grid.resolution,
                    self.grid.min_y + (y_idx + 0.5) * self.grid.resolution,
                    self.hazard_marker_height,
                ]
            )

        return hazards, np.asarray(hazard_points, dtype=np.float32)

    def pad_hazards(self, hazards: np.ndarray) -> np.ndarray:
        """Dilate hazard cells slightly around crater detections."""
        if self.hazard_padding_cells <= 0 or not hazards.any():
            return hazards

        padded = hazards.copy()
        hazard_indices = np.argwhere(hazards)
        for y_idx, x_idx in hazard_indices:
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

    def publish_outputs(
        self,
        input_header: Header,
        hazard_points: np.ndarray,
        hazard_cells: np.ndarray,
    ) -> None:
        """Publish the local crater hazards as a point cloud and grid."""
        header = Header()
        header.stamp = input_header.stamp
        header.frame_id = self.base_frame

        self.hazard_points_pub.publish(
            point_cloud2.create_cloud_xyz32(header, hazard_points.tolist())
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
            100 if occupied else 0 for occupied in hazard_cells.reshape(-1)
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
