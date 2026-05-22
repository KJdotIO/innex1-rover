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

"""Filter arena-wall point-cloud returns before autonomy consumers."""

from dataclasses import dataclass

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points_numpy
from std_msgs.msg import Header
from tf2_ros import ExtrapolationException, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


@dataclass(frozen=True)
class ArenaBounds:
    """Interior arena bounds plus a wall-exclusion margin."""

    min_x: float = -1.0
    max_x: float = 6.9
    min_y: float = -3.3
    max_y: float = 1.1
    wall_exclusion_margin_m: float = 0.35

    def __post_init__(self) -> None:
        """Validate that the legal area remains non-empty."""
        if self.min_x >= self.max_x:
            raise ValueError("arena_min_x must be less than arena_max_x")
        if self.min_y >= self.max_y:
            raise ValueError("arena_min_y must be less than arena_max_y")
        if self.wall_exclusion_margin_m < 0.0:
            raise ValueError("wall_exclusion_margin_m must be >= 0")
        if 2.0 * self.wall_exclusion_margin_m >= (self.max_x - self.min_x):
            raise ValueError("wall_exclusion_margin_m removes all legal arena width")
        if 2.0 * self.wall_exclusion_margin_m >= (self.max_y - self.min_y):
            raise ValueError("wall_exclusion_margin_m removes all legal arena height")

    @property
    def legal_min_x(self) -> float:
        """Lowest x coordinate allowed through the filter."""
        return self.min_x + self.wall_exclusion_margin_m

    @property
    def legal_max_x(self) -> float:
        """Highest x coordinate allowed through the filter."""
        return self.max_x - self.wall_exclusion_margin_m

    @property
    def legal_min_y(self) -> float:
        """Lowest y coordinate allowed through the filter."""
        return self.min_y + self.wall_exclusion_margin_m

    @property
    def legal_max_y(self) -> float:
        """Highest y coordinate allowed through the filter."""
        return self.max_y - self.wall_exclusion_margin_m


@dataclass(frozen=True)
class BoundaryFilterResult:
    """Filtered points and counts for diagnostics/evidence."""

    points: np.ndarray
    input_count: int
    kept_count: int
    rejected_count: int

    @property
    def reject_ratio(self) -> float:
        """Share of finite input points rejected by the boundary filter."""
        if self.input_count == 0:
            return 0.0
        return self.rejected_count / self.input_count


def transform_points(
    points: np.ndarray,
    translation_xyz: tuple[float, float, float],
    quaternion_xyzw: tuple[float, float, float, float],
) -> np.ndarray:
    """Apply a rigid transform to an Nx3 point array."""
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must be an Nx3 array")
    x, y, z, w = quaternion_xyzw
    norm = np.sqrt((x * x) + (y * y) + (z * z) + (w * w))
    if norm == 0.0:
        raise ValueError("quaternion must be non-zero")
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    rotation = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    translation = np.array(translation_xyz, dtype=np.float64)
    return (rotation @ points.T).T + translation


def filter_points_to_arena(
    points: np.ndarray, bounds: ArenaBounds
) -> BoundaryFilterResult:
    """Reject points outside the legal interior of the arena."""
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must be an Nx3 array")

    finite_mask = np.all(np.isfinite(points), axis=1)
    finite_points = points[finite_mask]
    if finite_points.shape[0] == 0:
        return BoundaryFilterResult(
            points=np.empty((0, 3), dtype=np.float32),
            input_count=0,
            kept_count=0,
            rejected_count=0,
        )

    legal_mask = (
        (finite_points[:, 0] >= bounds.legal_min_x)
        & (finite_points[:, 0] <= bounds.legal_max_x)
        & (finite_points[:, 1] >= bounds.legal_min_y)
        & (finite_points[:, 1] <= bounds.legal_max_y)
    )
    filtered = finite_points[legal_mask].astype(np.float32, copy=False)
    input_count = int(finite_points.shape[0])
    kept_count = int(filtered.shape[0])
    return BoundaryFilterResult(
        points=filtered,
        input_count=input_count,
        kept_count=kept_count,
        rejected_count=input_count - kept_count,
    )


class ArenaBoundaryFilterNode(Node):
    """Publish a wall-excluded point cloud for navigation/perception consumers."""

    def __init__(self):
        """Set up TF, cloud I/O, and diagnostics."""
        super().__init__("arena_boundary_filter")

        self.declare_parameter("input_topic", "/camera_front/points")
        self.declare_parameter(
            "output_topic",
            "/perception/arena_boundary/camera_front/points",
        )
        self.declare_parameter("source_name", "camera_front")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("arena_min_x", -1.0)
        self.declare_parameter("arena_max_x", 6.9)
        self.declare_parameter("arena_min_y", -3.3)
        self.declare_parameter("arena_max_y", 1.1)
        self.declare_parameter("wall_exclusion_margin_m", 0.35)
        self.declare_parameter("diagnostic_publish_hz", 1.0)
        self.declare_parameter("stale_timeout_s", 2.5)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.source_name = self.get_parameter("source_name").value
        self.target_frame = self.get_parameter("target_frame").value
        self.bounds = ArenaBounds(
            min_x=self.get_parameter("arena_min_x").value,
            max_x=self.get_parameter("arena_max_x").value,
            min_y=self.get_parameter("arena_min_y").value,
            max_y=self.get_parameter("arena_max_y").value,
            wall_exclusion_margin_m=self.get_parameter("wall_exclusion_margin_m").value,
        )
        self.stale_timeout_s = self.get_parameter("stale_timeout_s").value
        diagnostic_publish_hz = self.get_parameter("diagnostic_publish_hz").value

        if diagnostic_publish_hz <= 0.0:
            raise ValueError("diagnostic_publish_hz must be > 0")
        if self.stale_timeout_s <= 0.0:
            raise ValueError("stale_timeout_s must be > 0")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_result: BoundaryFilterResult | None = None
        self.last_cloud_time: Time | None = None
        self.last_error = ""

        self.cloud_pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos_profile_sensor_data,
        )
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray,
            "/diagnostics",
            10,
        )
        self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / diagnostic_publish_hz, self._publish_diagnostics)

        self.get_logger().info(
            f"Arena boundary filter {self.source_name}: {self.input_topic} -> "
            f"{self.output_topic} in {self.target_frame}; "
            f"legal x=[{self.bounds.legal_min_x:.2f}, "
            f"{self.bounds.legal_max_x:.2f}], "
            f"y=[{self.bounds.legal_min_y:.2f}, {self.bounds.legal_max_y:.2f}]"
        )

    def _cloud_callback(self, msg: PointCloud2) -> None:
        """Transform a point cloud to the arena frame and publish legal points."""
        tf = self._lookup_cloud_transform(msg)
        if tf is None:
            return

        points = read_points_numpy(msg, field_names=["x", "y", "z"], skip_nans=True)
        if points.size == 0:
            result = BoundaryFilterResult(
                points=np.empty((0, 3), dtype=np.float32),
                input_count=0,
                kept_count=0,
                rejected_count=0,
            )
        else:
            q = tf.transform.rotation
            t = tf.transform.translation
            points_target = transform_points(
                points,
                (t.x, t.y, t.z),
                (q.x, q.y, q.z, q.w),
            )
            result = filter_points_to_arena(points_target, self.bounds)

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.target_frame
        self.cloud_pub.publish(create_cloud_xyz32(header, result.points.tolist()))

        self.last_result = result
        self.last_cloud_time = self.get_clock().now()
        self.last_error = ""

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
        """Log and remember a throttled TF lookup failure."""
        self.last_error = str(error)
        self.get_logger().warning(
            f"TF lookup {msg.header.frame_id} -> {self.target_frame} failed: {error}",
            throttle_duration_sec=5.0,
        )

    def _publish_diagnostics(self) -> None:
        """Publish filter state and rejection counts for evidence bags."""
        status = DiagnosticStatus()
        status.name = f"perception/arena_boundary_filter/{self.source_name}"
        status.hardware_id = self.source_name

        if self.last_result is None or self.last_cloud_time is None:
            status.level = DiagnosticStatus.STALE
            status.message = "No filtered cloud published yet"
            age_s = float("inf")
        else:
            age_s = (self.get_clock().now() - self.last_cloud_time).nanoseconds / 1e9
            if age_s > self.stale_timeout_s:
                status.level = DiagnosticStatus.STALE
                status.message = "No fresh boundary-filtered cloud"
            elif self.last_error:
                status.level = DiagnosticStatus.WARN
                status.message = "TF lookup failure while filtering"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "Filtering wall/out-of-field points"

        result = self.last_result or BoundaryFilterResult(
            points=np.empty((0, 3), dtype=np.float32),
            input_count=0,
            kept_count=0,
            rejected_count=0,
        )
        status.values = [
            KeyValue(key="input_topic", value=self.input_topic),
            KeyValue(key="output_topic", value=self.output_topic),
            KeyValue(key="target_frame", value=self.target_frame),
            KeyValue(key="input_count", value=str(result.input_count)),
            KeyValue(key="kept_count", value=str(result.kept_count)),
            KeyValue(key="rejected_count", value=str(result.rejected_count)),
            KeyValue(key="reject_ratio", value=f"{result.reject_ratio:.3f}"),
            KeyValue(key="last_cloud_age_s", value=f"{age_s:.3f}"),
            KeyValue(key="legal_min_x", value=f"{self.bounds.legal_min_x:.3f}"),
            KeyValue(key="legal_max_x", value=f"{self.bounds.legal_max_x:.3f}"),
            KeyValue(key="legal_min_y", value=f"{self.bounds.legal_min_y:.3f}"),
            KeyValue(key="legal_max_y", value=f"{self.bounds.legal_max_y:.3f}"),
            KeyValue(
                key="wall_exclusion_margin_m",
                value=f"{self.bounds.wall_exclusion_margin_m:.3f}",
            ),
        ]
        if self.last_error:
            status.values.append(KeyValue(key="last_error", value=self.last_error))

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self.diagnostic_pub.publish(array)


def main(args=None):
    """Run the arena boundary filter node."""
    rclpy.init(args=args)
    node = ArenaBoundaryFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
