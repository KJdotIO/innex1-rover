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

"""Unit tests for the velocity gate logic."""

from lunabot_drivetrain.velocity_gate import VelocityGate
from lunabot_interfaces.msg import DrivetrainStatus


class TestVelocityGateAllowedStates:
    """Verify the allowed-state set for the gate."""

    def test_ready_is_allowed(self):
        assert DrivetrainStatus.STATE_READY in VelocityGate._ALLOWED_STATES

    def test_driving_is_allowed(self):
        assert DrivetrainStatus.STATE_DRIVING in VelocityGate._ALLOWED_STATES

    def test_fault_is_not_allowed(self):
        assert DrivetrainStatus.STATE_FAULT not in VelocityGate._ALLOWED_STATES

    def test_estop_is_not_allowed(self):
        assert DrivetrainStatus.STATE_ESTOP not in VelocityGate._ALLOWED_STATES

    def test_uninitialised_is_not_allowed(self):
        states = VelocityGate._ALLOWED_STATES
        assert DrivetrainStatus.STATE_UNINITIALISED not in states


class TestStallDetection:
    """Verify encoder stall detection in the drivetrain bridge."""

    def _make_bridge(self):
        from unittest.mock import MagicMock

        from lunabot_drivetrain.drivetrain_bridge import DrivetrainBridge
        bridge = object.__new__(DrivetrainBridge)
        bridge._stall_thresh = 0.15
        bridge._stall_min_vel = 0.05
        bridge._stall_timeout = 2.0
        bridge._stall_start_time = None
        bridge._wheel_velocity_rps = [0.0, 0.0, 0.0, 0.0]
        bridge._fault_code = DrivetrainStatus.FAULT_NONE
        bridge._state = DrivetrainStatus.STATE_DRIVING
        bridge.get_logger = MagicMock()
        return bridge

    def test_no_stall_when_below_threshold(self):
        bridge = self._make_bridge()
        bridge._check_encoder_stall(0.0, 0.10, 0.10)
        assert bridge._state == DrivetrainStatus.STATE_DRIVING

    def test_no_stall_when_velocity_present(self):
        bridge = self._make_bridge()
        bridge._wheel_velocity_rps = [0.1, 0.1, 0.1, 0.1]
        bridge._check_encoder_stall(0.0, 0.5, 0.5)
        assert bridge._state == DrivetrainStatus.STATE_DRIVING

    def test_stall_detected_after_timeout(self):
        bridge = self._make_bridge()
        bridge._check_encoder_stall(0.0, 0.5, 0.5)
        assert bridge._stall_start_time == 0.0
        bridge._check_encoder_stall(3.0, 0.5, 0.5)
        assert bridge._state == DrivetrainStatus.STATE_FAULT
        assert bridge._fault_code == (
            DrivetrainStatus.FAULT_ENCODER_STALL
        )

    def test_stall_resets_when_velocity_returns(self):
        bridge = self._make_bridge()
        bridge._check_encoder_stall(0.0, 0.5, 0.5)
        assert bridge._stall_start_time is not None
        bridge._wheel_velocity_rps = [0.1, 0.1, 0.1, 0.1]
        bridge._check_encoder_stall(1.0, 0.5, 0.5)
        assert bridge._stall_start_time is None
