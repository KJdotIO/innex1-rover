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

"""Unit tests for the Sabertooth Packetized Serial encoder."""

import pytest

from lunabot_drivetrain.sabertooth_serial import _pack_command, throttle_to_bytes


class TestPackCommand:
    """Verify the four-byte Packetized Serial frame encoding."""

    def test_checksum_calculation(self):
        frame = _pack_command(128, 0, 64)
        address, command, data, checksum = frame
        assert checksum == (128 + 0 + 64) & 0x7F

    def test_frame_length(self):
        frame = _pack_command(128, 4, 127)
        assert len(frame) == 4

    def test_zero_data_is_stop(self):
        frame = _pack_command(128, 0, 0)
        assert frame[2] == 0

    def test_address_out_of_range_raises(self):
        with pytest.raises(ValueError, match="address"):
            _pack_command(256, 0, 0)

    def test_data_out_of_range_raises(self):
        with pytest.raises(ValueError, match="data"):
            _pack_command(128, 0, 128)


class TestThrottleToBytes:
    """Verify throttle-to-serial-bytes conversion."""

    def test_full_forward_both_motors(self):
        payload = throttle_to_bytes(128, 1.0, 1.0)
        assert len(payload) == 8
        assert payload[1] == 0  # M1 forward command
        assert payload[2] == 127  # full speed
        assert payload[5] == 4  # M2 forward command
        assert payload[6] == 127

    def test_full_reverse_both_motors(self):
        payload = throttle_to_bytes(128, -1.0, -1.0)
        assert payload[1] == 1  # M1 reverse
        assert payload[2] == 127
        assert payload[5] == 5  # M2 reverse
        assert payload[6] == 127

    def test_stop(self):
        payload = throttle_to_bytes(128, 0.0, 0.0)
        assert payload[2] == 0
        assert payload[6] == 0

    def test_half_throttle(self):
        payload = throttle_to_bytes(128, 0.5, -0.5)
        assert payload[1] == 0  # M1 forward
        assert payload[2] == 63  # ~half of 127
        assert payload[5] == 5  # M2 reverse
        assert payload[6] == 63

    def test_clamps_above_one(self):
        payload = throttle_to_bytes(128, 2.0, -2.0)
        assert payload[2] == 127
        assert payload[6] == 127

    def test_mixed_direction(self):
        payload = throttle_to_bytes(129, 0.3, -0.7)
        assert payload[0] == 129  # address
        assert payload[1] == 0  # M1 forward
        assert payload[5] == 5  # M2 reverse

    def test_checksum_valid_for_all_frames(self):
        payload = throttle_to_bytes(128, 0.6, -0.4)
        for offset in (0, 4):
            addr, cmd, data, cksum = payload[offset:offset + 4]
            assert cksum == (addr + cmd + data) & 0x7F


class TestDrivetrainBridgeTwistConversion:
    """Verify the differential drive kinematics (twist → wheel throttles)."""

    def test_pure_forward(self):
        from lunabot_drivetrain.drivetrain_bridge import DrivetrainBridge

        bridge = object.__new__(DrivetrainBridge)
        bridge._track_width = 0.44
        bridge._wheel_radius = 0.065
        bridge._max_throttle = 1.0
        left, right = bridge._twist_to_wheel_speeds(0.3, 0.0)
        assert abs(left - right) < 0.01, "Pure forward should give equal wheels"
        assert left > 0.0

    def test_pure_rotation(self):
        from lunabot_drivetrain.drivetrain_bridge import DrivetrainBridge

        bridge = object.__new__(DrivetrainBridge)
        bridge._track_width = 0.44
        bridge._wheel_radius = 0.065
        bridge._max_throttle = 1.0
        left, right = bridge._twist_to_wheel_speeds(0.0, 1.0)
        assert left < 0.0, "Left wheel should reverse for CCW rotation"
        assert right > 0.0, "Right wheel should go forward for CCW rotation"

    def test_throttle_clamped(self):
        from lunabot_drivetrain.drivetrain_bridge import DrivetrainBridge

        bridge = object.__new__(DrivetrainBridge)
        bridge._track_width = 0.44
        bridge._wheel_radius = 0.065
        bridge._max_throttle = 0.5
        left, right = bridge._twist_to_wheel_speeds(10.0, 0.0)
        assert abs(left) <= 0.5
        assert abs(right) <= 0.5
