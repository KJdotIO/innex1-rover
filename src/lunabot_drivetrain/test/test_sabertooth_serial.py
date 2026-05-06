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

"""Unit tests for the Sabertooth serial encoders."""

import sys
import types

import pytest

import lunabot_drivetrain.sabertooth_serial as sabertooth_serial
from lunabot_drivetrain.sabertooth_serial import (
    _pack_command,
    simplified_throttle_to_bytes,
    throttle_to_bytes,
)

try:
    from lunabot_interfaces.msg import DrivetrainStatus, DrivetrainTelemetry
except ModuleNotFoundError:
    fake_msg = types.ModuleType("lunabot_interfaces.msg")

    class DrivetrainStatus:
        STATE_UNINITIALISED = 0
        STATE_READY = 1
        STATE_DRIVING = 2
        STATE_FAULT = 3
        STATE_ESTOP = 4
        FAULT_NONE = 0
        FAULT_CONTROLLER_OFFLINE = 2
        FAULT_ENCODER_STALL = 3

    class DrivetrainTelemetry:
        pass

    fake_msg.DrivetrainStatus = DrivetrainStatus
    fake_msg.DrivetrainTelemetry = DrivetrainTelemetry
    sys.modules.setdefault("lunabot_interfaces", types.ModuleType("lunabot_interfaces"))
    sys.modules["lunabot_interfaces.msg"] = fake_msg


class TestPackCommand:
    """Verify the four-byte Packet Serial frame encoding."""

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


class TestSimplifiedThrottleToBytes:
    """Verify Sabertooth Legacy Simplified Serial byte encoding."""

    def test_stop(self):
        assert simplified_throttle_to_bytes(0.0, 0.0) == bytes([64, 192])

    def test_full_forward_both_motors(self):
        assert simplified_throttle_to_bytes(1.0, 1.0) == bytes([127, 255])

    def test_full_reverse_both_motors(self):
        assert simplified_throttle_to_bytes(-1.0, -1.0) == bytes([1, 128])

    def test_half_throttle(self):
        assert simplified_throttle_to_bytes(0.5, -0.5) == bytes([96, 160])


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


class TestDrivetrainBridgeSerialDispatch:
    """Verify the bridge chooses the configured Sabertooth protocol."""

    def _make_bridge(self, protocol):
        from lunabot_drivetrain.drivetrain_bridge import DrivetrainBridge

        bridge = object.__new__(DrivetrainBridge)
        bridge._serial = object()
        bridge._serial_protocol = protocol
        bridge._addresses = [128, 129]
        return bridge

    def test_legacy_simplified_throttle_path(self, monkeypatch):
        calls = []

        monkeypatch.setattr(
            sabertooth_serial,
            "send_simplified_throttle",
            lambda port, left, right: calls.append((port, left, right)),
        )
        monkeypatch.setattr(
            sabertooth_serial,
            "send_throttle",
            lambda *args: pytest.fail(f"unexpected packet call: {args}"),
        )

        bridge = self._make_bridge("legacy_simplified")
        bridge._send_wheel_throttles(0.2, -0.1)

        assert calls == [(bridge._serial, 0.2, -0.1)]

    def test_packetized_throttle_path(self, monkeypatch):
        calls = []

        monkeypatch.setattr(
            sabertooth_serial,
            "send_simplified_throttle",
            lambda *args: pytest.fail(f"unexpected simplified call: {args}"),
        )
        monkeypatch.setattr(
            sabertooth_serial,
            "send_throttle",
            lambda port, address, left, right: calls.append(
                (port, address, left, right)
            ),
        )

        bridge = self._make_bridge("packetized")
        bridge._send_wheel_throttles(0.2, -0.1)

        assert calls == [
            (bridge._serial, 128, 0.2, -0.1),
            (bridge._serial, 129, 0.2, -0.1),
        ]

    def test_legacy_simplified_stop_path(self, monkeypatch):
        calls = []

        monkeypatch.setattr(
            sabertooth_serial,
            "send_simplified_stop",
            lambda port: calls.append(port),
        )
        monkeypatch.setattr(
            sabertooth_serial,
            "send_stop",
            lambda *args: pytest.fail(f"unexpected packet stop: {args}"),
        )

        bridge = self._make_bridge("simplified")
        bridge._send_stop()

        assert calls == [bridge._serial]

    def test_packetized_stop_path(self, monkeypatch):
        calls = []

        monkeypatch.setattr(
            sabertooth_serial,
            "send_simplified_stop",
            lambda *args: pytest.fail(f"unexpected simplified stop: {args}"),
        )
        monkeypatch.setattr(
            sabertooth_serial,
            "send_stop",
            lambda port, address: calls.append((port, address)),
        )

        bridge = self._make_bridge("packet_serial")
        bridge._send_stop()

        assert calls == [(bridge._serial, 128), (bridge._serial, 129)]
