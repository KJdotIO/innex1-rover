# Copyright 2026 Leicester Lunabotics Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Mission acceptance tests — no Gazebo required.

Launches the material action server stubs and mission_state_publisher,
runs one excavate→deposit cycle via the action clients, then asserts:
  - /mission/state reaches AUTONOMY_ACTIVE
  - /mission/cycle_count reaches 1
  - /mission/autonomy_mode publishes AUTONOMY
  - No SAFE_STOP occurs on the happy path
"""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.action import ActionClient
from std_msgs.msg import Int32, String

from lunabot_interfaces.action import Deposit, Excavate

# Accelerated stub durations so the test completes in seconds
_EXCAVATE_DURATION_S = 2.0
_DEPOSIT_DURATION_S = 2.0

# How long the test will wait for each step before failing
_STEP_TIMEOUT_S = 15.0


@pytest.mark.launch_test
def generate_test_description():
    """Bring up stub action servers and mission state publisher."""
    material_server = launch_ros.actions.Node(
        package="lunabot_control",
        executable="material_action_server",
        parameters=[{
            "excavate_nominal_duration_s": _EXCAVATE_DURATION_S,
            "deposit_nominal_duration_s": _DEPOSIT_DURATION_S,
        }],
    )

    mission_state_publisher = launch_ros.actions.Node(
        package="lunabot_mission",
        executable="mission_state_publisher",
    )

    return launch.LaunchDescription([
        material_server,
        mission_state_publisher,
        launch_testing.actions.ReadyToTest(),
    ])


class TestMissionAcceptance(unittest.TestCase):
    """Assert mission state transitions after one full excavate→deposit cycle."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_mission_acceptance")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin(self, timeout_s=0.1):
        rclpy.spin_once(self.node, timeout_sec=timeout_s)

    def _wait_for_future(self, future, timeout_s=_STEP_TIMEOUT_S):
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            self._spin()
        return future.done()

    def test_happy_path_one_cycle(self):
        """One excavate→deposit should yield cycle_count=1 and AUTONOMY_ACTIVE."""
        observed_states = []
        observed_modes = []
        cycle_count = [0]

        state_sub = self.node.create_subscription(
            String, "/mission/state",
            lambda msg: observed_states.append(msg.data), 10,
        )
        mode_sub = self.node.create_subscription(
            String, "/mission/autonomy_mode",
            lambda msg: observed_modes.append(msg.data), 10,
        )
        count_sub = self.node.create_subscription(
            Int32, "/mission/cycle_count",
            lambda msg: cycle_count.__setitem__(0, msg.data), 10,
        )

        excavate_client = ActionClient(self.node, Excavate, "/mission/excavate")
        deposit_client = ActionClient(self.node, Deposit, "/mission/deposit")

        try:
            self.assertTrue(
                excavate_client.wait_for_server(timeout_sec=10.0),
                "Excavate action server did not come up in time",
            )
            self.assertTrue(
                deposit_client.wait_for_server(timeout_sec=10.0),
                "Deposit action server did not come up in time",
            )

            # --- Excavate ---
            excavate_goal = Excavate.Goal()
            excavate_goal.mode = Excavate.Goal.MODE_AUTO
            excavate_goal.timeout_s = float(_STEP_TIMEOUT_S)
            excavate_goal.target_fill_fraction = 0.8
            excavate_goal.max_drive_speed_mps = 0.2

            future = excavate_client.send_goal_async(excavate_goal)
            self.assertTrue(self._wait_for_future(future), "Excavate goal send timed out")
            handle = future.result()
            self.assertTrue(handle.accepted, "Excavate goal was rejected")

            result_future = handle.get_result_async()
            self.assertTrue(
                self._wait_for_future(result_future), "Excavate result timed out"
            )
            self.assertTrue(
                result_future.result().result.success,
                f"Excavate failed: {result_future.result().result.failure_reason}",
            )

            # --- Deposit ---
            deposit_goal = Deposit.Goal()
            deposit_goal.mode = Deposit.Goal.MODE_AUTO
            deposit_goal.timeout_s = float(_STEP_TIMEOUT_S)
            deposit_goal.dump_duration_s = 1.0
            deposit_goal.require_close_after_dump = True

            future = deposit_client.send_goal_async(deposit_goal)
            self.assertTrue(self._wait_for_future(future), "Deposit goal send timed out")
            handle = future.result()
            self.assertTrue(handle.accepted, "Deposit goal was rejected")

            result_future = handle.get_result_async()
            self.assertTrue(
                self._wait_for_future(result_future), "Deposit result timed out"
            )
            self.assertTrue(
                result_future.result().result.success,
                f"Deposit failed: {result_future.result().result.failure_reason}",
            )

            # Allow mission_state_publisher 1 Hz tick to catch up
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                self._spin(0.1)

            # --- Assertions ---
            self.assertEqual(
                cycle_count[0], 1,
                f"Expected cycle_count=1, got {cycle_count[0]}",
            )
            self.assertIn(
                "AUTONOMY_ACTIVE", observed_states,
                f"State never reached AUTONOMY_ACTIVE, saw: {set(observed_states)}",
            )
            self.assertNotIn(
                "SAFE_STOP", observed_states,
                "Unexpected SAFE_STOP on happy path",
            )
            self.assertIn(
                "AUTONOMY", observed_modes,
                f"autonomy_mode never published AUTONOMY, saw: {set(observed_modes)}",
            )

        finally:
            self.node.destroy_subscription(state_sub)
            self.node.destroy_subscription(mode_sub)
            self.node.destroy_subscription(count_sub)
            excavate_client.destroy()
            deposit_client.destroy()
