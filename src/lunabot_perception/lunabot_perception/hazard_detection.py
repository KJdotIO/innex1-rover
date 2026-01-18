import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class HazardDetectionNode(Node):
    def __init__(self):
        super().__init__('hazard_detection_node')
        self.publisher = self.create_publisher(PointCloud2, '/hazards/front', 10)

        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera_front/points',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HazardDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()