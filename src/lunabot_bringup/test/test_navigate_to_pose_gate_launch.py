"""Integration tests for the NavigateToPose gate action forwarding."""

from __future__ import annotations

import os
from pathlib import Path
import sys
import unittest

from action_msgs.msg import GoalStatus
import launch
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch.actions import UnsetEnvironmentVariable
from launch_ros.actions import Node
import launch_testing.actions
from nav2_msgs.action import NavigateToPose
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node as RclpyNode

PUBLIC_ACTION_NAME = "/test_navigate_to_pose_gate"
INTERNAL_ACTION_NAME = "/test_navigate_to_pose"
TEST_ROS_DOMAIN_ID = str(100 + (os.getpid() % 100))

os.environ["ROS_DOMAIN_ID"] = TEST_ROS_DOMAIN_ID
os.environ.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
os.environ.pop("RMW_FASTRTPS_USE_QOS_FROM_XML", None)


@pytest.mark.launch_test
def generate_test_description():
    """Launch the gate and a tiny downstream action server."""
    test_server = ExecuteProcess(
        cmd=[
            sys.executable,
            str(Path(__file__).resolve().parent / "navigate_to_pose_test_server.py"),
        ],
        output="screen",
    )
    gate = Node(
        package="lunabot_bringup",
        executable="navigate_to_pose_gate",
        name="navigate_to_pose_gate",
        output="screen",
        parameters=[
            {
                "gate_enabled": False,
                "public_action_name": PUBLIC_ACTION_NAME,
                "internal_action_name": INTERNAL_ACTION_NAME,
            }
        ],
    )

    return (
        launch.LaunchDescription(
            [
                SetEnvironmentVariable("ROS_DOMAIN_ID", TEST_ROS_DOMAIN_ID),
                UnsetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE"),
                UnsetEnvironmentVariable("RMW_FASTRTPS_USE_QOS_FROM_XML"),
                test_server,
                gate,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "gate": gate,
            "test_server": test_server,
        },
    )


class _GateTestClient(RclpyNode):
    """Minimal action client for gate launch tests."""

    def __init__(self) -> None:
        super().__init__("navigate_to_pose_gate_test_client")
        self.client = ActionClient(
            self,
            NavigateToPose,
            PUBLIC_ACTION_NAME,
        )
        self.downstream_client = ActionClient(
            self,
            NavigateToPose,
            INTERNAL_ACTION_NAME,
        )


class TestNavigateToPoseGate(unittest.TestCase):
    """Exercise the gate action under a real launched node."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = _GateTestClient()
        self.assertTrue(self.node.client.wait_for_server(timeout_sec=5.0))
        self.assertTrue(self.node.downstream_client.wait_for_server(timeout_sec=5.0))

    def tearDown(self):
        self.node.destroy_node()

    def _send_goal(self, *, frame_id=""):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame_id

        send_goal_future = self.node.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=5.0)

        self.assertTrue(send_goal_future.done())
        goal_handle = send_goal_future.result()
        self.assertIsNotNone(goal_handle)
        self.assertTrue(goal_handle.accepted)
        return goal_handle

    def test_goal_completes_without_deadlocking(self):
        goal_handle = self._send_goal()

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=5.0)

        self.assertTrue(result_future.done())
        self.assertEqual(result_future.result().status, GoalStatus.STATUS_SUCCEEDED)

    def test_goal_cancel_reaches_terminal_state(self):
        goal_handle = self._send_goal()

        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=5.0)
        self.assertTrue(cancel_future.done())

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=5.0)

        self.assertTrue(result_future.done())
        self.assertEqual(result_future.result().status, GoalStatus.STATUS_CANCELED)

    def test_downstream_abort_is_returned(self):
        goal_handle = self._send_goal(frame_id="abort")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=5.0)

        self.assertTrue(result_future.done())
        self.assertEqual(result_future.result().status, GoalStatus.STATUS_ABORTED)
