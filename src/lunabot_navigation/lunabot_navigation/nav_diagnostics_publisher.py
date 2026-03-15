"""Publish navigation diagnostics: nearest obstacle, costmap density."""

import math

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float32


class NavDiagnosticsPublisher(Node):
    """Aggregate navigation health signals into a single diagnostics stream."""

    def __init__(self) -> None:
        """Initialise subscriptions and diagnostics publishers."""
        super().__init__("nav_diagnostics_publisher")

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self._nearest_obstacle = float("inf")
        self._obstacle_cells_near = 0
        self._robot_x = 0.0
        self._robot_y = 0.0

        diagnostics_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.nearest_pub = self.create_publisher(
            Float32, "/diagnostics/nearest_obstacle_m", diagnostics_qos
        )
        self.density_pub = self.create_publisher(
            Float32, "/diagnostics/obstacle_cells_near_robot", diagnostics_qos
        )

        self.create_subscription(
            Odometry, "/odometry/filtered", self._on_odom, sensor_qos
        )
        self.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self._on_local_costmap,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

        self.create_timer(2.0, self._publish_diagnostics)

    def _on_odom(self, msg: Odometry) -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        w = msg.info.width
        h = msg.info.height

        nearest = float("inf")
        cells_near = 0
        radius = 1.5  # metres

        for j in range(h):
            for i in range(w):
                cost = msg.data[j * w + i]
                if cost < 80:
                    continue
                cx = ox + (i + 0.5) * res
                cy = oy + (j + 0.5) * res
                d = math.hypot(cx - self._robot_x, cy - self._robot_y)
                if d < nearest:
                    nearest = d
                if d < radius:
                    cells_near += 1

        self._nearest_obstacle = nearest
        self._obstacle_cells_near = cells_near

    def _publish_diagnostics(self) -> None:
        nearest_msg = Float32()
        nearest_msg.data = float(self._nearest_obstacle)
        self.nearest_pub.publish(nearest_msg)

        density_msg = Float32()
        density_msg.data = float(self._obstacle_cells_near)
        self.density_pub.publish(density_msg)


def main(args=None) -> None:
    """Run the navigation diagnostics publisher."""
    rclpy.init(args=args)
    node = NavDiagnosticsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
