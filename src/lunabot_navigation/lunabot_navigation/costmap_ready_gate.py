"""Block Nav2 goals until the costmap has obstacle data from point clouds.

Subscribes to /camera_front/points and publishes a Bool on
/nav/costmap_ready once a minimum number of messages have arrived.
Other nodes (or the test harness) should check this before sending goals.

Also provides a service /nav/wait_for_costmap that blocks until ready.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class CostmapReadyGate(Node):

    def __init__(self):
        super().__init__("costmap_ready_gate")
        self.declare_parameter("min_messages", 3)
        self.declare_parameter("topic", "/camera_front/points")

        self.min_messages = self.get_parameter("min_messages").value
        topic = self.get_parameter("topic").value
        self.msg_count = 0
        self.ready = False

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(PointCloud2, topic, self._on_points, sensor_qos)
        self.pub = self.create_publisher(Bool, "/nav/costmap_ready", 10)
        self.create_service(Trigger, "/nav/wait_for_costmap", self._srv_callback)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f"Costmap gate: waiting for {self.min_messages} point cloud messages"
        )

    def _on_points(self, msg):
        if self.ready:
            return
        self.msg_count += 1
        if self.msg_count >= self.min_messages:
            self.ready = True
            self.get_logger().info(
                "Costmap populated -- ready for navigation goals"
            )

    def _publish_status(self):
        msg = Bool()
        msg.data = self.ready
        self.pub.publish(msg)

    def _srv_callback(self, request, response):
        response.success = self.ready
        response.message = (
            "Costmap ready" if self.ready
            else f"Waiting for point clouds ({self.msg_count}/{self.min_messages})"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CostmapReadyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
