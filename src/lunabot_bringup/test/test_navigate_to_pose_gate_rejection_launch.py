"""Launch test: gate rejects goals when gate_enabled=True and no status is published."""

from __future__ import annotations

import atexit
import os
import time
import unittest
from pathlib import Path

import launch
import launch_testing.actions
import pytest
import rclpy
from launch.actions import (
    SetEnvironmentVariable,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch_ros.actions import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node as RclpyNode

PUBLIC_ACTION_NAME = "/test_rejection_navigate_to_pose_gate"
INTERNAL_ACTION_NAME = "/test_rejection_navigate_to_pose"
TEST_ROS_DOMAIN_LOCK_FD = None
TEST_ROS_DOMAIN_LOCK_PATH = None


def _reserve_test_domain():
    """Reserve a ROS domain id for this test run on the current machine."""
    global TEST_ROS_DOMAIN_LOCK_FD
    global TEST_ROS_DOMAIN_LOCK_PATH

    domain_ids = list(range(1, 102)) + list(range(215, 233))
    start_index = (os.getpid() + time.time_ns() + 7777) % len(domain_ids)

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
    """Launch the gate with gate_enabled=True and no status publisher."""
    gate = Node(
        package="lunabot_bringup",
        executable="navigate_to_pose_gate",
        name="navigate_to_pose_gate_rejection",
        output="screen",
        parameters=[
            {
                "gate_enabled": True,
                "public_action_name": PUBLIC_ACTION_NAME,
                "internal_action_name": INTERNAL_ACTION_NAME,
                "readiness_timeout_s": 0.1,
            }
        ],
    )

    return (
        launch.LaunchDescription(
            [
                SetEnvironmentVariable("ROS_DOMAIN_ID", TEST_ROS_DOMAIN_ID),
                UnsetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE"),
                UnsetEnvironmentVariable("RMW_FASTRTPS_USE_QOS_FROM_XML"),
                gate,
                TimerAction(
                    period=2.0,
                    actions=[launch_testing.actions.ReadyToTest()],
                ),
            ]
        ),
        {"gate": gate},
    )


class _RejectionTestClient(RclpyNode):
    """Minimal action client for gate rejection tests."""

    def __init__(self) -> None:
        super().__init__("navigate_to_pose_gate_rejection_test_client")
        self.client = ActionClient(
            self,
            NavigateToPose,
            PUBLIC_ACTION_NAME,
        )


class TestNavigateToPoseGateRejectsWithNoStatus(unittest.TestCase):
    """Verify the gate rejects goals when gate_enabled=True and no status is published."""

    @classmethod
    def setUpClass(cls):
        """Initialise rclpy with the reserved test domain."""
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
        """Shutdown rclpy and restore the environment."""
        if rclpy.ok():
            rclpy.shutdown()
        for name, value in cls._original_env.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value

        _release_test_domain_reservation()

    def setUp(self):
        """Create a test client node."""
        self.node = _RejectionTestClient()
        self.assertTrue(self.node.client.wait_for_server(timeout_sec=10.0))

    def tearDown(self):
        """Destroy the test client node."""
        self.node.destroy_node()

    def test_gate_rejects_goal_when_no_status_published(self):
        """Gate with gate_enabled=True and no status publisher must reject goals."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = ""

        send_goal_future = self.node.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=5.0)

        self.assertTrue(send_goal_future.done())
        goal_handle = send_goal_future.result()
        self.assertIsNotNone(goal_handle)
        self.assertFalse(
            goal_handle.accepted,
            "Gate should reject the goal when no localisation status is available.",
        )
