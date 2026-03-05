import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, LookupException
from lunabot_interfaces.srv import GetReadiness
import sys

class ReadinessGate(Node):
    def __init__(self):
        super().__init__('readiness_gate')
        
        # 1. DECLARE PARAMETERS
        # These will be overwritten by the values in the .yaml file
        self.declare_parameter('max_position_variance', 0.0625)
        self.declare_parameter('max_yaw_variance', 0.002)
        self.declare_parameter('topic_timeout', 3.0) # Increased for VirtualBox stability
        self.declare_parameter('required_frames', ['map', 'odom', 'base_link'])

        # 2. SETUP SUBSCRIPTIONS & TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        # 3. SETUP THE SERVICE FOR STATE MANAGER
        self.srv = self.create_service(GetReadiness, 'check_localisation_ready', self.handle_ready_request)

        self.last_msg_time = None
        self.latest_odom = None

    def odom_callback(self, msg):
        self.last_msg_time = self.get_clock().now()
        self.latest_odom = msg

    def handle_ready_request(self, request, response):
        """Service callback to return pass/fail and reason"""
        is_ready, reason = self.evaluate_gate()
        response.is_ready = is_ready
        response.message = reason
        return response

    def evaluate_gate(self):
        now = self.get_clock().now()
        
        # 1. Requirement: Check if data has EVER been received
        if self.latest_odom is None or self.last_msg_time is None:
            return False, "FAIL: No odom data received yet"

        # 2. Requirement: Check freshness/timeouts
        timeout = self.get_parameter('topic_timeout').value
        elapsed = (now - self.last_msg_time).nanoseconds / 1e9
        
        if elapsed > timeout:
            return False, f"FAIL: Data is stale ({elapsed:.1f}s old)"

        # 3. Requirement: Check TF tree availability
        frames = self.get_parameter('required_frames').value
        try:
            # Check if map -> base_link exists
            self.tf_buffer.lookup_transform(frames[0], frames[2], rclpy.time.Time())
        except Exception:
            return False, f"FAIL: TF Tree missing {frames[0]} to {frames[2]}"

        # 4. Requirement: Check covariance thresholds
        cov = self.latest_odom.pose.covariance
        max_pos_var = self.get_parameter('max_position_variance').value
        # Index 0 is X variance, 7 is Y variance
        if cov[0] > max_pos_var or cov[7] > max_pos_var:
            return False, f"FAIL: High covariance ({max(cov[0], cov[7]):.4f})"

        return True, "PASS: Localization ready"

def main(args=None):
    rclpy.init(args=args)
    node = ReadinessGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node
