#!/usr/bin/env python3
"""Automated nav stability test harness.

Sends a sequence of goals through the Nav2 action server, records
per-goal success/fail, time-to-goal, and prints a pass/fail summary.

Waits for costmap readiness (/nav/costmap_ready) before sending the
first goal to prevent driving blind through obstacles.

Usage:
    # Launch everything first:
    #   ros2 launch lunabot_bringup simulation.launch.py
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

# Arena: 7.9x4.4m centered at (0,0). X: -3.95..3.95, Y: -2.2..2.2
# Start zone: top-left around (-2.95, 1.1)
# Obstacle zone: roughly X -1.95 to 1.20
# Excavation zone: X > 1.20
# Goals traverse the arena through the obstacle slalom
GOALS = [
    {"name": "start_zone_edge",    "x": -2.0, "y":  0.5, "yaw": 0.0},
    {"name": "obstacle_entry",     "x": -1.0, "y": -0.2, "yaw": 0.0},
    {"name": "obstacle_mid",       "x":  0.0, "y":  0.5, "yaw": 0.0},
    {"name": "excavation_entry",   "x":  1.5, "y":  0.0, "yaw": 0.0},
    {"name": "return_to_start",    "x": -2.5, "y":  1.0, "yaw": math.pi},
]

GOAL_TIMEOUT_SEC = 120.0
COSTMAP_WAIT_SEC = 60.0
SETTLE_WAIT_SEC = 10.0


@dataclass
class GoalResult:
    name: str
    success: bool = False
    elapsed_sec: float = 0.0
    status: int = 0
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
        self._costmap_ready = False
        self.create_subscription(
            Bool, "/nav/costmap_ready", self._on_costmap_ready, sensor_qos
        )
        self.results: list[GoalResult] = []

    def _on_costmap_ready(self, msg: Bool):
        self._costmap_ready = msg.data

    def wait_for_costmap(self) -> bool:
        self.get_logger().info("Waiting for costmap to populate...")
        deadline = time.monotonic() + COSTMAP_WAIT_SEC
        while not self._costmap_ready and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=1.0)
        if self._costmap_ready:
            self.get_logger().info("Costmap ready")
        else:
            self.get_logger().warn(
                f"Costmap not ready after {COSTMAP_WAIT_SEC}s, proceeding anyway"
            )
        return self._costmap_ready

    def wait_for_server(self, timeout_sec=30.0) -> bool:
        self.get_logger().info("Waiting for navigate_to_pose action server...")
        return self._action_client.wait_for_server(timeout_sec)

    def send_goal_and_wait(self, goal_dict: dict) -> GoalResult:
        result = GoalResult(name=goal_dict["name"])

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

        self.wait_for_costmap()

        self.get_logger().info(
            f"Settling for {SETTLE_WAIT_SEC}s to let EKF stabilize..."
        )
        t_settle = time.monotonic() + SETTLE_WAIT_SEC
        while time.monotonic() < t_settle:
            rclpy.spin_once(self, timeout_sec=1.0)

        for goal in GOALS:
            result = self.send_goal_and_wait(goal)
            self.results.append(result)
            t_pause = time.monotonic() + 3.0
            while time.monotonic() < t_pause:
                rclpy.spin_once(self, timeout_sec=0.5)

        return self.print_summary()

    def print_summary(self) -> bool:
        print("\n" + "=" * 60)
        print("NAV STABILITY TEST RESULTS")
        print("=" * 60)
        print(f"{'Goal':<25} {'Result':<10} {'Time(s)':<10} {'Error'}")
        print("-" * 60)

        all_pass = True
        for r in self.results:
            status = "PASS" if r.success else "FAIL"
            if not r.success:
                all_pass = False
            print(
                f"{r.name:<25} {status:<10} {r.elapsed_sec:<10.1f} {r.error}"
            )

        print("-" * 60)
        passed = sum(1 for r in self.results if r.success)
        total = len(self.results)
        verdict = "PASS" if all_pass else "FAIL"
        print(f"Overall: {passed}/{total} goals succeeded -- {verdict}")
        print("=" * 60 + "\n")
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
