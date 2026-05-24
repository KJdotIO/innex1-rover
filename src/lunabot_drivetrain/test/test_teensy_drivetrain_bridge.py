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

"""Behaviour tests for the Teensy drivetrain bridge safety state."""

import time
import types

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist

from lunabot_drivetrain.teensy_drivetrain_bridge import TeensyDrivetrainBridge
from lunabot_drivetrain.teensy_protocol import (
    DrivetrainTelemetry,
    TeensyStatus,
    pack_drivetrain_telemetry,
    pack_status,
)
from lunabot_interfaces.msg import DrivetrainStatus


def _bridge_for_command_tests():
    bridge = object.__new__(TeensyDrivetrainBridge)
    bridge._state = DrivetrainStatus.STATE_READY
    bridge._fault_code = DrivetrainStatus.FAULT_NONE
    bridge._teensy_state = DrivetrainStatus.STATE_READY
    bridge._teensy_fault_code = DrivetrainStatus.FAULT_NONE
    bridge._teensy_status_fault_code = DrivetrainStatus.FAULT_NONE
    bridge._teensy_telemetry_fault_code = DrivetrainStatus.FAULT_NONE
    bridge._ros_estop_active = False
    bridge._ros_motion_inhibited = False
    bridge._teensy_estop_active = False
    bridge._teensy_motion_inhibited = False
    bridge._motion_inhibited = False
    bridge._estop_active = False
    bridge._controller_online = [True, True]
    bridge._last_cmd_time = time.monotonic()
    bridge._last_twist = Twist()
    bridge._last_twist.linear.x = 0.2
    bridge._track_width = 0.44
    bridge._wheel_radius = 0.065
    bridge._max_throttle = 1.0
    bridge._cmd_timeout = 0.5
    bridge._last_odom_time = None
    bridge._odom_x = 0.0
    bridge._odom_y = 0.0
    bridge._odom_yaw = 0.0
    return bridge


def test_teensy_fault_is_not_cleared_by_next_drive_command():
    bridge = _bridge_for_command_tests()

    bridge._apply_status(
        pack_status(
            TeensyStatus(
                state=DrivetrainStatus.STATE_FAULT,
                fault_code=DrivetrainStatus.FAULT_OVERCURRENT,
                estop_active=False,
                motion_inhibited=False,
                controller_online=(True, True),
            )
        )
    )
    command = bridge._build_drive_command(time.monotonic())

    assert bridge._state == DrivetrainStatus.STATE_FAULT
    assert bridge._fault_code == DrivetrainStatus.FAULT_OVERCURRENT
    assert command.enabled is False
    assert command.left == 0.0
    assert command.right == 0.0


def test_odometry_is_published_from_teensy_wheel_velocity():
    bridge = _bridge_for_command_tests()
    published = []
    bridge._odom_pub = types.SimpleNamespace(publish=published.append)
    bridge._wheel_velocity_rps = [1.0, 1.0, 1.0, 1.0]
    bridge._last_odom_time = time.monotonic() - 1.0

    bridge._publish_odometry(Time())

    assert len(published) == 1
    odom = published[0]
    assert odom.header.frame_id == "odom"
    assert odom.child_frame_id == "base_footprint"
    assert odom.pose.pose.position.x > 0.3
    assert (
        abs(odom.twist.twist.linear.x - (2.0 * 3.141592653589793 * 0.065))
        < 0.01
    )


def test_telemetry_without_fault_does_not_clear_status_fault():
    bridge = _bridge_for_command_tests()

    bridge._apply_status(
        pack_status(
            TeensyStatus(
                state=DrivetrainStatus.STATE_FAULT,
                fault_code=DrivetrainStatus.FAULT_OVERCURRENT,
                estop_active=False,
                motion_inhibited=False,
                controller_online=(True, True),
            )
        )
    )
    bridge._apply_telemetry(
        pack_drivetrain_telemetry(
            DrivetrainTelemetry(
                encoder_ticks=(0, 0, 0, 0),
                wheel_velocity_rps=(0.0, 0.0, 0.0, 0.0),
                fault_code=DrivetrainStatus.FAULT_NONE,
                estop_active=False,
                motion_inhibited=False,
                controller_online=(True, True),
            )
        )
    )

    assert bridge._state == DrivetrainStatus.STATE_FAULT
    assert bridge._fault_code == DrivetrainStatus.FAULT_OVERCURRENT
