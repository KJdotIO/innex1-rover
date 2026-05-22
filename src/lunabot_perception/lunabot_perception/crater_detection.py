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

"""Crater (negative obstacle) detection from depth camera point clouds."""

import numpy as np
import rclpy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points_numpy
from std_msgs.msg import Header
from tf2_ros import ExtrapolationException, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def _rotation_matrix_from_quaternion(
    x: float, y: float, z: float, w: float
) -> np.ndarray:
    """Return a 3x3 rotation matrix from a normalised or unnormalised quaternion."""
    norm = np.sqrt((x * x) + (y * y) + (z * z) + (w * w))
    if norm == 0.0:
        raise ValueError("quaternion must be non-zero")
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def _binary_dilation(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    """Dilate a 2D boolean mask without adding a SciPy runtime dependency."""
    if radius_cells <= 0 or not np.any(mask):
        return mask

    padded = np.pad(mask, radius_cells, mode="constant", constant_values=False)
    dilated = np.zeros_like(mask, dtype=bool)
    kernel_width = (2 * radius_cells) + 1
    for row_offset in range(kernel_width):
        for col_offset in range(kernel_width):
            row_stop = row_offset + mask.shape[0]
            col_stop = col_offset + mask.shape[1]
            dilated |= padded[row_offset:row_stop, col_offset:col_stop]
    return dilated


class CraterDetectionNode(Node):
    """Detect craters by maintaining an elevation grid and flagging depressions."""

    def __init__(self):
        """Set up elevation grid, TF listener, publishers and subscribers."""
        super().__init__("crater_detection_node")

        # --- Declare parameters ---
        self.declare_parameter("grid_resolution", 0.05)
        self.declare_parameter("grid_width", 8.0)
        self.declare_parameter("grid_height", 5.0)
        self.declare_parameter("grid_origin_x", -1.0)
        self.declare_parameter("grid_origin_y", -3.5)
        self.declare_parameter("depth_threshold", 0.08)
        self.declare_parameter("min_points_per_cell", 2)
        self.declare_parameter("update_rate", 5.0)
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("ground_percentile", 80.0)
        self.declare_parameter("inflation_cells", 2)
        self.declare_parameter("accumulator_decay", 0.95)
        self.declare_parameter(
            "input_cloud_topic",
            "/perception/arena_boundary/camera_front/points",
        )
        # Fixed ground Z for competition arenas with known flat floor
        # Set to NaN to use automatic percentile-based estimation
        self.declare_parameter("fixed_ground_z", float("nan"))

        # --- Read parameters ---
        self.resolution = self.get_parameter("grid_resolution").value
        self.grid_width_m = self.get_parameter("grid_width").value
        self.grid_height_m = self.get_parameter("grid_height").value
        self.origin_x = self.get_parameter("grid_origin_x").value
        self.origin_y = self.get_parameter("grid_origin_y").value
        self.depth_threshold = self.get_parameter("depth_threshold").value
        self.min_points = self.get_parameter("min_points_per_cell").value
        self.target_frame = self.get_parameter("target_frame").value
        self.ground_percentile = self.get_parameter("ground_percentile").value
        self.inflation_cells = self.get_parameter("inflation_cells").value
        self.decay = self.get_parameter("accumulator_decay").value
        self.input_cloud_topic = self.get_parameter("input_cloud_topic").value
        self.fixed_ground_z = self.get_parameter("fixed_ground_z").value

        if self.resolution <= 0.0:
            raise ValueError("grid_resolution must be > 0")
        if self.grid_width_m <= 0.0 or self.grid_height_m <= 0.0:
            raise ValueError("grid_width and grid_height must be > 0")
        if self.min_points < 1:
            raise ValueError("min_points_per_cell must be >= 1")
        if self.decay <= 0.0 or self.decay > 1.0:
            raise ValueError("accumulator_decay must be in (0, 1]")

        # Grid dimensions in cells
        self.grid_w = int(self.grid_width_m / self.resolution)
        self.grid_h = int(self.grid_height_m / self.resolution)

        # Elevation accumulators (float64 for both to support decay)
        self._z_sum = np.zeros((self.grid_h, self.grid_w), dtype=np.float64)
        self._z_count = np.zeros((self.grid_h, self.grid_w), dtype=np.float64)

        # Pre-build dilation kernel for crater inflation
        self._inflation_radius_cells = max(0, int(self.inflation_cells))

        # --- TF2 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Subscriber ---
        self.create_subscription(
            PointCloud2,
            self.input_cloud_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )

        # --- Publishers ---
        # VOLATILE QoS so StaticLayer re-reads every publish
        grid_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.grid_pub = self.create_publisher(OccupancyGrid, "/crater_grid", grid_qos)

        self.debug_cloud_pub = self.create_publisher(
            PointCloud2, "/crater_points_debug", qos_profile_sensor_data
        )

        # --- Publish timer ---
        rate = self.get_parameter("update_rate").value
        if rate <= 0.0:
            raise ValueError("update_rate must be > 0")
        self.create_timer(1.0 / rate, self._publish_grid)

        self.get_logger().info(
            f"Crater detection: {self.grid_w}x{self.grid_h} grid "
            f"@ {self.resolution}m, depth_threshold={self.depth_threshold}m, "
            f"decay={self.decay}, inflation={self.inflation_cells} cells, "
            f"input={self.input_cloud_topic}"
        )

    def _cloud_callback(self, msg: PointCloud2):
        """Transform incoming cloud to odom frame and accumulate elevation."""
        tf = self._lookup_cloud_transform(msg)
        if tf is None:
            return

        points = read_points_numpy(msg, field_names=["x", "y", "z"], skip_nans=True)
        if points.size == 0:
            return

        valid = np.all(np.isfinite(points), axis=1)
        points = points[valid]
        if points.shape[0] == 0:
            return

        q = tf.transform.rotation
        t = tf.transform.translation
        rot = _rotation_matrix_from_quaternion(q.x, q.y, q.z, q.w)
        trans = np.array([t.x, t.y, t.z])
        points_odom = (rot @ points.T).T + trans

        col = ((points_odom[:, 0] - self.origin_x) / self.resolution).astype(np.int32)
        row = ((points_odom[:, 1] - self.origin_y) / self.resolution).astype(np.int32)
        z = points_odom[:, 2]

        mask = (col >= 0) & (col < self.grid_w) & (row >= 0) & (row < self.grid_h)
        col = col[mask]
        row = row[mask]
        z = z[mask]

        if col.size == 0:
            return

        # Decay old data so recent observations dominate
        self._z_sum *= self.decay
        self._z_count *= self.decay

        np.add.at(self._z_sum, (row, col), z)
        np.add.at(self._z_count, (row, col), 1.0)

    def _lookup_cloud_transform(self, msg: PointCloud2):
        """Return the transform for a cloud, tolerating small future TF offsets."""
        cloud_time = Time.from_msg(msg.header.stamp)
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                cloud_time,
                timeout=Duration(seconds=0.0),
            )
        except ExtrapolationException as e:
            if "future" not in str(e).lower():
                self._log_tf_lookup_failure(msg, e)
                return None

            try:
                latest_tf = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=0.0),
                )
            except TransformException as latest_error:
                self._log_tf_lookup_failure(msg, latest_error)
                return None

            self.get_logger().warning(
                f"TF lookup {msg.header.frame_id} -> {self.target_frame} "
                f"needed a future transform; using latest available transform: {e}",
                throttle_duration_sec=5.0,
            )
            return latest_tf
        except TransformException as e:
            self._log_tf_lookup_failure(msg, e)
            return None

    def _log_tf_lookup_failure(
        self, msg: PointCloud2, error: TransformException
    ) -> None:
        """Log a throttled TF lookup failure for an incoming cloud."""
        self.get_logger().warning(
            f"TF lookup {msg.header.frame_id} -> {self.target_frame} failed: {error}",
            throttle_duration_sec=5.0,
        )

    def _publish_grid(self):
        """Compute crater grid and publish OccupancyGrid."""
        observed = self._z_count >= self.min_points
        if not np.any(observed):
            return

        elevation = np.where(
            observed, self._z_sum / np.maximum(self._z_count, 1.0), np.nan
        )

        # Ground reference: fixed value or auto-percentile
        if np.isfinite(self.fixed_ground_z):
            ground_z = self.fixed_ground_z
        else:
            observed_elevations = elevation[observed]
            ground_z = np.percentile(observed_elevations, self.ground_percentile)

        crater_mask = observed & (elevation < (ground_z - self.depth_threshold))

        # Inflate using proper dilation (no wraparound)
        crater_mask = _binary_dilation(crater_mask, self._inflation_radius_cells)

        # Build OccupancyGrid
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = self.target_frame
        grid_msg.info = MapMetaData()
        grid_msg.info.resolution = float(self.resolution)
        grid_msg.info.width = self.grid_w
        grid_msg.info.height = self.grid_h
        grid_msg.info.origin.position.x = self.origin_x
        grid_msg.info.origin.position.y = self.origin_y
        grid_msg.info.origin.position.z = 0.0

        data = np.full(self.grid_h * self.grid_w, 0, dtype=np.int8)
        data[crater_mask.ravel()] = 100  # LETHAL
        data[~observed.ravel()] = -1  # UNKNOWN
        grid_msg.data = data.tolist()

        self.grid_pub.publish(grid_msg)

        # Debug: publish crater centroids as point cloud for RViz
        crater_indices = np.argwhere(crater_mask)
        if crater_indices.size > 0:
            cx = (
                crater_indices[:, 1] * self.resolution
                + self.origin_x
                + self.resolution / 2
            )
            cy = (
                crater_indices[:, 0] * self.resolution
                + self.origin_y
                + self.resolution / 2
            )
            cz = elevation[crater_mask]
            cz = np.where(np.isfinite(cz), cz, 0.0)
            debug_pts = np.column_stack([cx, cy, cz]).astype(np.float32)
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.target_frame
            self.debug_cloud_pub.publish(create_cloud_xyz32(header, debug_pts))


def main(args=None):
    """Run the crater detection node."""
    rclpy.init(args=args)
    node = CraterDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
