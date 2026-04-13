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

"""Crater (negative obstacle) detection from depth camera point clouds.

Builds a rolling 2.5D elevation grid from the front camera's point cloud,
identifies cells significantly below the local ground plane, and publishes
an OccupancyGrid marking those cells as lethal for Nav2 costmap consumption.
"""

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy, create_cloud_xyz32
from std_msgs.msg import Header
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class CraterDetectionNode(Node):
    """Detect craters by maintaining an elevation grid and flagging depressions."""

    def __init__(self):
        """Set up elevation grid, TF listener, publishers and subscribers."""
        super().__init__('crater_detection_node')

        # --- Declare parameters ---
        self.declare_parameter('grid_resolution', 0.05)
        self.declare_parameter('grid_width', 8.0)
        self.declare_parameter('grid_height', 5.0)
        self.declare_parameter('grid_origin_x', -1.0)
        self.declare_parameter('grid_origin_y', -3.5)
        self.declare_parameter('depth_threshold', 0.12)
        self.declare_parameter('min_points_per_cell', 3)
        self.declare_parameter('update_rate', 5.0)
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('ground_percentile', 80.0)
        self.declare_parameter('inflation_cells', 2)

        # --- Read parameters ---
        self.resolution = self.get_parameter('grid_resolution').value
        self.grid_width_m = self.get_parameter('grid_width').value
        self.grid_height_m = self.get_parameter('grid_height').value
        self.origin_x = self.get_parameter('grid_origin_x').value
        self.origin_y = self.get_parameter('grid_origin_y').value
        self.depth_threshold = self.get_parameter('depth_threshold').value
        self.min_points = self.get_parameter('min_points_per_cell').value
        self.target_frame = self.get_parameter('target_frame').value
        self.ground_percentile = self.get_parameter('ground_percentile').value
        self.inflation_cells = self.get_parameter('inflation_cells').value

        # Grid dimensions in cells
        self.grid_w = int(self.grid_width_m / self.resolution)
        self.grid_h = int(self.grid_height_m / self.resolution)

        # Elevation accumulators: sum of Z values and count per cell
        self._z_sum = np.zeros((self.grid_h, self.grid_w), dtype=np.float64)
        self._z_count = np.zeros(
            (self.grid_h, self.grid_w), dtype=np.int32
        )

        # --- TF2 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Subscriber ---
        self.create_subscription(
            PointCloud2,
            '/camera_front/points',
            self._cloud_callback,
            10,
        )

        # --- Publishers ---
        # Latched QoS so Nav2 StaticLayer picks up the latest grid
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.grid_pub = self.create_publisher(
            OccupancyGrid, '/crater_grid', latched_qos
        )

        # Debug: publish crater points as PointCloud2 for RViz
        self.debug_cloud_pub = self.create_publisher(
            PointCloud2, '/crater_points_debug', 10
        )

        # --- Publish timer ---
        rate = self.get_parameter('update_rate').value
        self.create_timer(1.0 / rate, self._publish_grid)

        self.get_logger().info(
            f'Crater detection: {self.grid_w}x{self.grid_h} grid '
            f'@ {self.resolution}m, depth threshold {self.depth_threshold}m'
        )

    def _cloud_callback(self, msg: PointCloud2):
        """Transform incoming cloud to odom frame and accumulate elevation."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.2),
            )
        except Exception:
            return

        # Read as (N, 3) array
        points = read_points_numpy(
            msg, field_names=['x', 'y', 'z'], skip_nans=True
        )
        if points.size == 0:
            return

        # Remove infinites
        valid = np.all(np.isfinite(points), axis=1)
        points = points[valid]
        if points.shape[0] == 0:
            return

        # Manual TF: rotate then translate (avoids do_transform_cloud bugs)
        q = tf.transform.rotation
        t = tf.transform.translation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        trans = np.array([t.x, t.y, t.z])
        points_odom = (rot @ points.T).T + trans

        # Bin into grid cells
        col = ((points_odom[:, 0] - self.origin_x) / self.resolution).astype(
            np.int32
        )
        row = ((points_odom[:, 1] - self.origin_y) / self.resolution).astype(
            np.int32
        )
        z = points_odom[:, 2]

        # Mask to valid grid bounds
        mask = (
            (col >= 0) & (col < self.grid_w)
            & (row >= 0) & (row < self.grid_h)
        )
        col = col[mask]
        row = row[mask]
        z = z[mask]

        if col.size == 0:
            return

        # Accumulate per-cell Z values
        np.add.at(self._z_sum, (row, col), z)
        np.add.at(self._z_count, (row, col), 1)

    def _publish_grid(self):
        """Compute crater grid and publish OccupancyGrid."""
        observed = self._z_count >= self.min_points
        if not np.any(observed):
            return

        # Mean elevation per cell
        elevation = np.where(
            observed, self._z_sum / np.maximum(self._z_count, 1), np.nan
        )

        # Ground reference: high percentile of observed elevations
        # (most of the terrain is flat ground, craters are the minority)
        observed_elevations = elevation[observed]
        ground_z = np.percentile(observed_elevations, self.ground_percentile)

        # Crater cells: observed, and significantly below ground
        crater_mask = observed & (elevation < (ground_z - self.depth_threshold))

        # Inflate crater cells to add a safety margin
        if self.inflation_cells > 0:
            inflated = np.copy(crater_mask)
            k = self.inflation_cells
            for dr in range(-k, k + 1):
                for dc in range(-k, k + 1):
                    if dr == 0 and dc == 0:
                        continue
                    shifted = np.roll(np.roll(crater_mask, dr, axis=0), dc, axis=1)
                    inflated |= shifted
            crater_mask = inflated

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
        # Mark unobserved as unknown (-1) so Nav2 doesn't treat them as free
        data[~observed.ravel()] = -1
        grid_msg.data = data.tolist()

        self.grid_pub.publish(grid_msg)

        # Debug: publish crater centroids as point cloud for RViz
        crater_indices = np.argwhere(crater_mask)
        if crater_indices.size > 0:
            cx = (crater_indices[:, 1] * self.resolution
                  + self.origin_x + self.resolution / 2)
            cy = (crater_indices[:, 0] * self.resolution
                  + self.origin_y + self.resolution / 2)
            cz = elevation[crater_mask]
            cz = np.where(np.isfinite(cz), cz, 0.0)
            debug_pts = np.column_stack([cx, cy, cz]).astype(np.float32)
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.target_frame
            self.debug_cloud_pub.publish(
                create_cloud_xyz32(header, debug_pts)
            )


def main(args=None):
    """Run the crater detection node."""
    rclpy.init(args=args)
    node = CraterDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
