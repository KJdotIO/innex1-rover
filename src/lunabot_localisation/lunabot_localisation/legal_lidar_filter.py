"""ROS node for field-preserving legal LiDAR localisation filtering."""

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import ExtrapolationException, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from lunabot_localisation.legal_lidar_filter_core import (
    ArenaBounds,
    LegalLidarFilterResult,
    filter_cloud_to_legal_bounds,
)


class LegalLidarFilterNode(Node):
    """Publish wall-excluded LiDAR clouds without dropping Ouster point fields."""

    def __init__(self):
        """Set up TF, cloud I/O, and diagnostics."""
        super().__init__("legal_lidar_filter")

        self.declare_parameter("input_topic", "/ouster/points")
        self.declare_parameter("output_topic", "/localisation/lidar/points_legal")
        self.declare_parameter("source_name", "ouster")
        self.declare_parameter("mask_frame", "odom")
        self.declare_parameter("arena_min_x", -1.0)
        self.declare_parameter("arena_max_x", 6.9)
        self.declare_parameter("arena_min_y", -3.3)
        self.declare_parameter("arena_max_y", 1.1)
        self.declare_parameter("wall_exclusion_margin_m", 0.35)
        self.declare_parameter("diagnostic_publish_hz", 1.0)
        self.declare_parameter("stale_timeout_s", 2.5)
        self.declare_parameter("tf_lookup_timeout_s", 0.05)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.source_name = self.get_parameter("source_name").value
        self.mask_frame = self.get_parameter("mask_frame").value
        self.bounds = ArenaBounds(
            min_x=self.get_parameter("arena_min_x").value,
            max_x=self.get_parameter("arena_max_x").value,
            min_y=self.get_parameter("arena_min_y").value,
            max_y=self.get_parameter("arena_max_y").value,
            wall_exclusion_margin_m=self.get_parameter("wall_exclusion_margin_m").value,
        )
        self.stale_timeout_s = self.get_parameter("stale_timeout_s").value
        self.tf_lookup_timeout_s = self.get_parameter("tf_lookup_timeout_s").value
        diagnostic_publish_hz = self.get_parameter("diagnostic_publish_hz").value

        if diagnostic_publish_hz <= 0.0:
            raise ValueError("diagnostic_publish_hz must be > 0")
        if self.stale_timeout_s <= 0.0:
            raise ValueError("stale_timeout_s must be > 0")
        if self.tf_lookup_timeout_s < 0.0:
            raise ValueError("tf_lookup_timeout_s must be >= 0")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_result: LegalLidarFilterResult | None = None
        self.last_cloud_time: Time | None = None
        self.last_error = ""

        self.cloud_pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos_profile_sensor_data,
        )
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / diagnostic_publish_hz, self._publish_diagnostics)

        self.get_logger().info(
            f"Legal LiDAR filter {self.source_name}: {self.input_topic} -> "
            f"{self.output_topic}; mask in {self.mask_frame}; output frame stays "
            f"sensor-native; legal x=[{self.bounds.legal_min_x:.2f}, "
            f"{self.bounds.legal_max_x:.2f}], y=[{self.bounds.legal_min_y:.2f}, "
            f"{self.bounds.legal_max_y:.2f}]"
        )

    def _cloud_callback(self, msg: PointCloud2) -> None:
        """Filter a LiDAR cloud while preserving its original point layout."""
        tf = self._lookup_cloud_transform(msg)
        if tf is None:
            return

        q = tf.transform.rotation
        t = tf.transform.translation
        try:
            result = filter_cloud_to_legal_bounds(
                msg,
                self.bounds,
                (t.x, t.y, t.z),
                (q.x, q.y, q.z, q.w),
            )
        except ValueError as error:
            self.last_error = str(error)
            self.get_logger().warning(
                f"Failed to filter legal LiDAR cloud: {error}",
                throttle_duration_sec=5.0,
            )
            return

        filtered_msg = PointCloud2()
        filtered_msg.header.stamp = msg.header.stamp
        filtered_msg.header.frame_id = msg.header.frame_id
        filtered_msg.height = result.cloud.height
        filtered_msg.width = result.cloud.width
        filtered_msg.fields = result.cloud.fields
        filtered_msg.is_bigendian = result.cloud.is_bigendian
        filtered_msg.point_step = result.cloud.point_step
        filtered_msg.row_step = result.cloud.row_step
        filtered_msg.is_dense = result.cloud.is_dense
        filtered_msg.data = result.cloud.data
        self.cloud_pub.publish(filtered_msg)

        self.last_result = result
        self.last_cloud_time = self.get_clock().now()
        self.last_error = ""

    def _lookup_cloud_transform(self, msg: PointCloud2):
        """Return the transform used only to decide legal arena membership."""
        cloud_time = Time.from_msg(msg.header.stamp)
        try:
            return self.tf_buffer.lookup_transform(
                self.mask_frame,
                msg.header.frame_id,
                cloud_time,
                timeout=Duration(seconds=self.tf_lookup_timeout_s),
            )
        except ExtrapolationException as error:
            if "future" not in str(error).lower():
                self._log_tf_lookup_failure(msg, error)
                return None
            try:
                latest_tf = self.tf_buffer.lookup_transform(
                    self.mask_frame,
                    msg.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=self.tf_lookup_timeout_s),
                )
            except TransformException as latest_error:
                self._log_tf_lookup_failure(msg, latest_error)
                return None
            self.get_logger().warning(
                f"TF lookup {msg.header.frame_id} -> {self.mask_frame} needed a "
                f"future transform; using latest available transform: {error}",
                throttle_duration_sec=5.0,
            )
            return latest_tf
        except TransformException as error:
            self._log_tf_lookup_failure(msg, error)
            return None

    def _log_tf_lookup_failure(
        self, msg: PointCloud2, error: TransformException
    ) -> None:
        """Log and remember a throttled TF lookup failure."""
        self.last_error = str(error)
        self.get_logger().warning(
            f"TF lookup {msg.header.frame_id} -> {self.mask_frame} failed: {error}",
            throttle_duration_sec=5.0,
        )

    def _publish_diagnostics(self) -> None:
        """Publish filter state and rejection counts for evidence bags."""
        status = DiagnosticStatus()
        status.name = f"localisation/legal_lidar_filter/{self.source_name}"
        status.hardware_id = self.source_name

        if self.last_result is None or self.last_cloud_time is None:
            status.level = DiagnosticStatus.STALE
            status.message = "No legal LiDAR cloud published yet"
            age_s = float("inf")
        else:
            age_s = (self.get_clock().now() - self.last_cloud_time).nanoseconds / 1e9
            if age_s > self.stale_timeout_s:
                status.level = DiagnosticStatus.STALE
                status.message = "No fresh legal LiDAR cloud"
            elif self.last_error:
                status.level = DiagnosticStatus.WARN
                status.message = "Legal LiDAR filtering warning"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "Filtering LiDAR wall/out-of-field points"

        result = self.last_result
        status.values = [
            KeyValue(key="input_topic", value=self.input_topic),
            KeyValue(key="output_topic", value=self.output_topic),
            KeyValue(key="mask_frame", value=self.mask_frame),
            KeyValue(key="raw_count", value=str(result.raw_count if result else 0)),
            KeyValue(key="finite_count", value=str(result.finite_count if result else 0)),
            KeyValue(key="kept_count", value=str(result.kept_count if result else 0)),
            KeyValue(
                key="rejected_count",
                value=str(result.rejected_count if result else 0),
            ),
            KeyValue(
                key="reject_ratio",
                value=f"{result.reject_ratio:.3f}" if result else "0.000",
            ),
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
    """Run the legal LiDAR filter node."""
    rclpy.init(args=args)
    node = LegalLidarFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
