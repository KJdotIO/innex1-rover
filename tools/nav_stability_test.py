#!/usr/bin/env python3
"""Automated nav stability test harness.

Sends a sequence of goals through the Nav2 action server, records
per-goal success/fail, time-to-goal, recovery count, and prints a
pass/fail summary.

Usage:
    # First launch sim + navigation in another terminal:
    #   ros2 launch lunabot_simulation moon_yard.launch.py
    #   ros2 launch lunabot_bringup navigation.launch.py
    # Then run this test:
    python3 tools/nav_stability_test.py
"""

import math
import sys
import time
from dataclasses import dataclass

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

# Arena-aware goals: verified clear of all rocks in moon_yard.sdf
# Rocks at: (1.95,0.1) (3.25,-1.6) (2.1,-2.5) (3.8,0.5) (5.75,0.4) (6.15,-2.1) (5.2,-0.8)
# Rover spawns at (0, 0, 0.5)
GOALS = [
    {"name": "start_zone_south", "x": 0.5, "y": -1.0, "yaw": -math.pi / 2},
    {"name": "obstacle_passage", "x": 2.8, "y": -0.5, "yaw": 0.0},
    {"name": "deep_obstacle_zone", "x": 4.5, "y": -1.8, "yaw": math.pi},
    {"name": "return_mid", "x": 1.5, "y": -1.2, "yaw": math.pi},
    {"name": "back_to_start", "x": 0.3, "y": -0.3, "yaw": math.pi / 2},
]

GOAL_TIMEOUT_SEC = 120.0
SETTLE_WAIT_SEC = 25.0


@dataclass
class GoalResult:
    name: str
    success: bool = False
    elapsed_sec: float = 0.0
    status: int = 0
    vo_samples: int = 0
    vo_healthy_samples: int = 0
    error: str = ""


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class NavStabilityTest(Node):
    def __init__(self):
        super().__init__("nav_stability_test")
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._vo_healthy = False
        self._vo_samples = 0
        self._vo_healthy_count = 0
        self.create_subscription(
            Bool, "/visual_odometry/healthy", self._on_vo_health, sensor_qos
        )

        self.results: list[GoalResult] = []

    def _on_vo_health(self, msg: Bool):
        self._vo_samples += 1
        if msg.data:
            self._vo_healthy_count += 1
        self._vo_healthy = msg.data

    def wait_for_server(self, timeout_sec=30.0) -> bool:
        self.get_logger().info("Waiting for navigate_to_pose action server...")
        return self._action_client.wait_for_server(timeout_sec)

    def send_goal_and_wait(self, goal_dict: dict) -> GoalResult:
        result = GoalResult(name=goal_dict["name"])
        self._vo_samples = 0
        self._vo_healthy_count = 0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(goal_dict["x"])
        goal_msg.pose.pose.position.y = float(goal_dict["y"])
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = yaw_to_quaternion(goal_dict["yaw"])

        self.get_logger().info(
            f"Sending goal '{goal_dict['name']}' -> "
            f"({goal_dict['x']:.2f}, {goal_dict['y']:.2f})"
        )

        t0 = time.monotonic()
        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            result.error = "goal_rejected"
            self.get_logger().error(f"Goal '{goal_dict['name']}' was rejected")
            return result

        self.get_logger().info(f"Goal '{goal_dict['name']}' accepted, waiting...")
        result_future = goal_handle.get_result_async()

        deadline = time.monotonic() + GOAL_TIMEOUT_SEC
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)

        elapsed = time.monotonic() - t0
        result.elapsed_sec = elapsed
        result.vo_samples = self._vo_samples
        result.vo_healthy_samples = self._vo_healthy_count

        if result_future.done():
            nav_result = result_future.result()
            result.status = nav_result.status
            if nav_result.status == GoalStatus.STATUS_SUCCEEDED:
                result.success = True
                self.get_logger().info(
                    f"Goal '{goal_dict['name']}' SUCCEEDED in {elapsed:.1f}s"
                )
            else:
                result.error = f"status={nav_result.status}"
                self.get_logger().warn(
                    f"Goal '{goal_dict['name']}' FAILED status={nav_result.status} "
                    f"in {elapsed:.1f}s"
                )
        else:
            result.error = "timeout"
            self.get_logger().error(
                f"Goal '{goal_dict['name']}' TIMED OUT after {GOAL_TIMEOUT_SEC}s"
            )
            goal_handle.cancel_goal_async()

        return result

    def run_test_sequence(self):
        if not self.wait_for_server():
            self.get_logger().fatal("Action server not available, aborting")
            return False

        self.get_logger().info(
            f"Settling for {SETTLE_WAIT_SEC}s to let EKF/VO stabilize..."
        )
        t_settle = time.monotonic() + SETTLE_WAIT_SEC
        while time.monotonic() < t_settle:
            rclpy.spin_once(self, timeout_sec=1.0)

        for goal in GOALS:
            result = self.send_goal_and_wait(goal)
            self.results.append(result)
            # Brief pause between goals
            t_pause = time.monotonic() + 3.0
            while time.monotonic() < t_pause:
                rclpy.spin_once(self, timeout_sec=0.5)

        return self.print_summary()

    def print_summary(self) -> bool:
        print("\n" + "=" * 72)
        print("NAV STABILITY TEST RESULTS")
        print("=" * 72)
        print(
            f"{'Goal':<25} {'Result':<10} {'Time(s)':<10} "
            f"{'VO Pass%':<10} {'Error'}"
        )
        print("-" * 72)

        all_pass = True
        for r in self.results:
            vo_pct = (
                f"{100.0 * r.vo_healthy_samples / r.vo_samples:.0f}%"
                if r.vo_samples > 0
                else "N/A"
            )
            status = "PASS" if r.success else "FAIL"
            if not r.success:
                all_pass = False
            print(
                f"{r.name:<25} {status:<10} {r.elapsed_sec:<10.1f} "
                f"{vo_pct:<10} {r.error}"
            )

        print("-" * 72)
        passed = sum(1 for r in self.results if r.success)
        total = len(self.results)
        verdict = "PASS" if all_pass else "FAIL"
        print(f"Overall: {passed}/{total} goals succeeded -- {verdict}")
        print("=" * 72 + "\n")
        return all_pass


def main():
    rclpy.init()
    node = NavStabilityTest()
    try:
        success = node.run_test_sequence()
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted")
        success = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
