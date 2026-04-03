"""Tiny downstream action server used by navigate-to-pose gate tests."""

from __future__ import annotations

import time

from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

ACTION_NAME = "/test_navigate_to_pose"


class FakeNavigateToPoseServer(Node):
    """Serve minimal NavigateToPose goals for gate integration tests."""

    def __init__(self) -> None:
        super().__init__("fake_navigate_to_pose_server")
        self._callback_group = ReentrantCallbackGroup()
        self._server = ActionServer(
            self,
            NavigateToPose,
            ACTION_NAME,
            execute_callback=self._execute_goal,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._callback_group,
        )

    def destroy_node(self):
        """Destroy the action server before node teardown."""
        self._server.destroy()
        super().destroy_node()

    def _on_goal(self, _goal_request) -> GoalResponse:
        """Accept all test goals."""
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        """Allow cancellation during the fake execution loop."""
        return CancelResponse.ACCEPT

    def _execute_goal(self, goal_handle):
        """Succeed, abort, or cancel based on the test goal."""
        result = NavigateToPose.Result()

        if goal_handle.request.pose.header.frame_id == "abort":
            goal_handle.abort()
            return result

        for _ in range(20):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return result
            time.sleep(0.05)

        goal_handle.succeed()
        return result


def main(args=None) -> None:
    """Run the fake downstream NavigateToPose server."""
    rclpy.init(args=args)
    node = FakeNavigateToPoseServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
