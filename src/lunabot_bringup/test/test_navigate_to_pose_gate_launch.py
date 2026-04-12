"""Integration tests for the NavigateToPose gate action forwarding."""

from __future__ import annotations

import atexit
import os
import sys
import time
import unittest
from pathlib import Path

import launch
import launch_testing.actions
import pytest
import rclpy
from action_msgs.msg import GoalStatus
from launch.actions import (
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch_ros.actions import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node as RclpyNode

PUBLIC_ACTION_NAME = "/test_navigate_to_pose_gate"
INTERNAL_ACTION_NAME = "/test_navigate_to_pose"
TEST_ROS_DOMAIN_LOCK_FD = None
TEST_ROS_DOMAIN_LOCK_PATH = None


def _reserve_test_domain():
    """Reserve a ROS domain id for this test run on the current machine."""
    global TEST_ROS_DOMAIN_LOCK_FD
    global TEST_ROS_DOMAIN_LOCK_PATH

    domain_ids = list(range(1, 102)) + list(range(215, 233))
    start_index = (os.getpid() + time.time_ns()) % len(domain_ids)

    for offset in range(len(domain_ids)):
        domain_id = domain_ids[(start_index + offset) % len(domain_ids)]
        lock_path = Path(f"/tmp/lunabot_ros_domain_{domain_id}.lock")
        try:
            lock_fd = os.open(lock_path, os.O_CREAT | os.O_EXCL | os.O_RDWR)
        except FileExistsError:
            continue
        TEST_ROS_DOMAIN_LOCK_FD = lock_fd
        TEST_ROS_DOMAIN_LOCK_PATH = lock_path
        return str(domain_id)

    raise RuntimeError("No free ROS domain id available for launch test")


def _release_test_domain_reservation():
    """Release any reserved ROS domain id lock for this test run."""
    global TEST_ROS_DOMAIN_LOCK_FD
    global TEST_ROS_DOMAIN_LOCK_PATH

    if TEST_ROS_DOMAIN_LOCK_FD is not None:
        os.close(TEST_ROS_DOMAIN_LOCK_FD)
        TEST_ROS_DOMAIN_LOCK_FD = None

    if TEST_ROS_DOMAIN_LOCK_PATH is not None:
        TEST_ROS_DOMAIN_LOCK_PATH.unlink(missing_ok=True)
        TEST_ROS_DOMAIN_LOCK_PATH = None


TEST_ROS_DOMAIN_ID = _reserve_test_domain()
atexit.register(_release_test_domain_reservation)


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
                TimerAction(
                    period=2.0,
                    actions=[launch_testing.actions.ReadyToTest()],
                ),
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
        cls._original_env = {
            "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID"),
            "FASTRTPS_DEFAULT_PROFILES_FILE": os.environ.get(
                "FASTRTPS_DEFAULT_PROFILES_FILE"
            ),
            "RMW_FASTRTPS_USE_QOS_FROM_XML": os.environ.get(
                "RMW_FASTRTPS_USE_QOS_FROM_XML"
            ),
        }
        os.environ["ROS_DOMAIN_ID"] = TEST_ROS_DOMAIN_ID
        os.environ.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
        os.environ.pop("RMW_FASTRTPS_USE_QOS_FROM_XML", None)
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()
        for name, value in cls._original_env.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value

        _release_test_domain_reservation()

    def setUp(self):
        self.node = _GateTestClient()
        self.assertTrue(self.node.client.wait_for_server(timeout_sec=10.0))
        self.assertTrue(self.node.downstream_client.wait_for_server(timeout_sec=10.0))

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
