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

"""Unit tests for the Teensy motor IO serial protocol."""

import pytest

from lunabot_drivetrain.teensy_protocol import (
    DriveCommand,
    DrivetrainTelemetry,
    Frame,
    MessageType,
    ProtocolError,
    TeensyStatus,
    decode_frame,
    encode_frame,
    pack_drive_command,
    pack_drivetrain_telemetry,
    pack_status,
    unpack_drive_command,
    unpack_drivetrain_telemetry,
    unpack_status,
)


def test_frame_round_trip_preserves_type_sequence_and_body():
    frame = Frame(
        msg_type=MessageType.DRIVE_COMMAND,
        sequence=42,
        body=b"\x01\x00\x7f\x00\x80",
    )

    decoded = decode_frame(encode_frame(frame))

    assert decoded == frame


def test_corrupted_frame_is_rejected():
    encoded = bytearray(encode_frame(Frame(MessageType.HEARTBEAT, 1, b"")))
    encoded[2] ^= 0x55

    with pytest.raises(ProtocolError):
        decode_frame(bytes(encoded))


def test_drive_command_body_round_trips_and_clamps():
    command = DriveCommand(
        left=2.0,
        right=-0.25,
        enabled=True,
        estop_active=False,
        motion_inhibited=True,
    )

    decoded = unpack_drive_command(pack_drive_command(command))

    assert decoded.left == 1.0
    assert decoded.right == -0.25
    assert decoded.enabled is True
    assert decoded.estop_active is False
    assert decoded.motion_inhibited is True


def test_status_body_round_trips():
    status = TeensyStatus(
        state=2,
        fault_code=0,
        estop_active=False,
        motion_inhibited=True,
        controller_online=(True, False),
    )

    assert unpack_status(pack_status(status)) == status


def test_telemetry_body_round_trips_scaled_values():
    telemetry = DrivetrainTelemetry(
        encoder_ticks=(1, -2, 3, -4),
        wheel_velocity_rps=(0.1234, -0.5, 1.0, 0.0),
        fault_code=4,
        estop_active=True,
        motion_inhibited=False,
        controller_online=(True, True),
    )

    decoded = unpack_drivetrain_telemetry(pack_drivetrain_telemetry(telemetry))

    assert decoded.encoder_ticks == telemetry.encoder_ticks
    assert decoded.wheel_velocity_rps == (0.123, -0.5, 1.0, 0.0)
    assert decoded.fault_code == telemetry.fault_code
    assert decoded.estop_active is True
    assert decoded.motion_inhibited is False
    assert decoded.controller_online == (True, True)
