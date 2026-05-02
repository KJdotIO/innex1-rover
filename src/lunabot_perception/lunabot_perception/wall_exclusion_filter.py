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

"""Republish point clouds with perimeter wall returns removed (map-frame crop)."""

from __future__ import annotations

import numpy as np

from lunabot_perception.wall_exclusion_geometry import (
    apply_rigid_transform,
    compute_interior_bounds,
    interior_xy_mask,
)
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points
from tf2_ros import Buffer, TransformException, TransformListener


class WallExclusionFilter(Node):
    """Drop points outside an axis-aligned interior rectangle in *map* frame."""

    def __init__(self) -> None:
        super().__init__("wall_exclusion_filter")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("arena_min_x", -1.0)
        self.declare_parameter("arena_max_x", 6.9)
        self.declare_parameter("arena_min_y", -3.3)
        self.declare_parameter("arena_max_y", 1.1)
        self.declare_parameter("inset_margin_m", 0.2)
        self.declare_parameter("max_points_per_cloud", 45000)
        self.declare_parameter("transform_timeout_sec", 0.15)
        self.declare_parameter("input_topics", ["/ouster/points"])
        self.declare_parameter(
            "output_topics",
            ["/perception/autonomy/ouster/points"],
        )

        self._map_frame = str(self.get_parameter("map_frame").value)
        min_x = float(self.get_parameter("arena_min_x").value)
        max_x = float(self.get_parameter("arena_max_x").value)
        min_y = float(self.get_parameter("arena_min_y").value)
        max_y = float(self.get_parameter("arena_max_y").value)
        inset = float(self.get_parameter("inset_margin_m").value)
        self._max_points = int(self.get_parameter("max_points_per_cloud").value)
        _tfs = float(self.get_parameter("transform_timeout_sec").value)
        self._tf_timeout = Duration(seconds=_tfs)

        if self._max_points < 1000:
            raise ValueError("max_points_per_cloud must be >= 1000")
        if inset < 0.0:
            raise ValueError("inset_margin_m must be non-negative")
        self._bounds = compute_interior_bounds(min_x, max_x, min_y, max_y, inset)
        inner_min_x, inner_max_x, inner_min_y, inner_max_y = self._bounds

        inputs = list(self.get_parameter("input_topics").value)
        outputs = list(self.get_parameter("output_topics").value)
        if len(inputs) != len(outputs):
            raise ValueError("input_topics and output_topics must have the same length")
        if len(inputs) == 0:
            raise ValueError("At least one input/output topic pair is required")
        for topic in inputs + outputs:
            if not topic or not str(topic).startswith("/"):
                raise ValueError(f"Invalid topic name: {topic!r}")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._publishers = []
        for index, out_topic in enumerate(outputs):
            publisher = self.create_publisher(PointCloud2, str(out_topic), qos_profile_sensor_data)
            self._publishers.append(publisher)
            input_topic = str(inputs[index])
            self.create_subscription(
                PointCloud2,
                input_topic,
                self._make_callback(index),
                qos_profile_sensor_data,
            )
            self.get_logger().info(
                f"Wall exclusion stream {index}: {input_topic} -> {out_topic} "
                f"(map interior x=[{inner_min_x:.2f},{inner_max_x:.2f}], "
                f"y=[{inner_min_y:.2f},{inner_max_y:.2f}])",
            )

    def _make_callback(self, stream_index: int):
        """Return a bounded callback for the given stream index."""

        def _cb(msg: PointCloud2) -> None:
            self._on_cloud(msg, stream_index)

        return _cb

    def _on_cloud(self, msg: PointCloud2, stream_index: int) -> None:
        """Transform cloud to map, crop to interior, republish."""
        source_frame = msg.header.frame_id
        if not source_frame:
            self.get_logger().warning("PointCloud2 missing frame_id; skipping")
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=self._tf_timeout,
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"TF {self._map_frame} <- {source_frame} failed: {exc}",
            )
            return

        xyz_points = list(read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if len(xyz_points) == 0:
            out = PointCloud2()
            out.header = msg.header
            out.header.frame_id = self._map_frame
            self._publishers[stream_index].publish(out)
            return

        points = np.asarray(xyz_points, dtype=np.float64)

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        try:
            points = apply_rigid_transform(
                np.asarray(points, dtype=np.float64),
                (translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w),
            )
        except ValueError as exc:
            self.get_logger().warning(f"Point-cloud transform failed: {exc}")
            return

        valid = np.all(np.isfinite(points), axis=1)
        points = points[valid]
        row_count = int(points.shape[0])
        if row_count == 0:
            empty = PointCloud2()
            empty.header = msg.header
            empty.header.frame_id = self._map_frame
            self._publishers[stream_index].publish(empty)
            return

        if row_count > self._max_points:
            step = int(np.ceil(row_count / float(self._max_points)))
            points = points[::step, :]
            self.get_logger().debug(
                f"Downsampled cloud from {row_count} to {points.shape[0]} points (cap)",
            )

        px = points[:, 0]
        py = points[:, 1]
        mask = interior_xy_mask(px, py, self._bounds)
        filtered = points[mask]
        if filtered.shape[0] == 0:
            out = PointCloud2()
            out.header = msg.header
            out.header.frame_id = self._map_frame
            self._publishers[stream_index].publish(out)
            return

        header = msg.header
        header.frame_id = self._map_frame
        out_cloud = create_cloud_xyz32(header, filtered.astype(np.float32))
        self._publishers[stream_index].publish(out_cloud)


def main() -> None:
    """Run the wall exclusion filter node."""
    rclpy.init()
    node = WallExclusionFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
