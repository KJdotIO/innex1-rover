"""Readiness gate node for mission precheck.

Exposes /mission/check_readiness (std_srvs/Trigger).
Returns success=True only when:
  - /odometry/filtered is publishing
  - /hazards/front is publishing
  - /camera_front/points is publishing
  - /mission/excavate action server is available
  - /mission/deposit action server is available
  - EKF position covariance is below threshold
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

from lunabot_interfaces.action import Excavate, Deposit

EKF_COV_THRESHOLD = 0.5
TOPIC_TIMEOUT_S = 5.0


class ReadinessGate(Node):
    def __init__(self):
        super().__init__("readiness_gate")

        self._odom_received = False
        self._hazards_received = False
        self._camera_received = False
        self._ekf_covariance = float("inf")

        self.create_subscription(Odometry, "/odometry/filtered", self._odom_cb, 10)
        self.create_subscription(PointCloud2, "/hazards/front", self._hazards_cb, 10)
        self.create_subscription(PointCloud2, "/camera_front/points", self._camera_cb, 10)

        self._excavate_client = ActionClient(self, Excavate, "/mission/excavate")
        self._deposit_client = ActionClient(self, Deposit, "/mission/deposit")

        self.create_service(Trigger, "/mission/check_readiness", self._check_readiness_cb)

        self.get_logger().info("Readiness gate online — waiting for /mission/check_readiness calls")

    def _odom_cb(self, msg: Odometry):
        self._odom_received = True
        self._ekf_covariance = msg.pose.covariance[0]

    def _hazards_cb(self, _msg: PointCloud2):
        self._hazards_received = True

    def _camera_cb(self, _msg: PointCloud2):
        self._camera_received = True

    def _check_readiness_cb(self, _request, response):
        failures = []

        if not self._odom_received:
            failures.append("/odometry/filtered not publishing")

        if not self._hazards_received:
            failures.append("/hazards/front not publishing")

        if not self._camera_received:
            failures.append("/camera_front/points not publishing")

        if self._ekf_covariance > EKF_COV_THRESHOLD:
            failures.append(
                f"EKF covariance too high ({self._ekf_covariance:.3f} > {EKF_COV_THRESHOLD})"
            )

        if not self._excavate_client.server_is_ready():
            failures.append("/mission/excavate action server not available")

        if not self._deposit_client.server_is_ready():
            failures.append("/mission/deposit action server not available")

        if failures:
            response.success = False
            response.message = "; ".join(failures)
            self.get_logger().warn(f"Readiness check FAILED: {response.message}")
        else:
            response.success = True
            response.message = "All checks passed"
            self.get_logger().info("Readiness check PASSED")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ReadinessGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
