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

"""Unit tests for the deposition bridge logic."""

from unittest.mock import MagicMock

from rclpy.action import GoalResponse

from lunabot_interfaces.action import Deposit


class TestDepositResult:
    """Verify result building from the deposition bridge."""

    def test_success_result_fields(self):
        from lunabot_control.deposition_bridge import DepositionBridge
        r = DepositionBridge._result(True, 0, "", 5.0)
        assert r.success is True
        assert r.reason_code == 0
        assert r.duration_s == 5.0
        assert r.residual_fill_fraction_estimate < 0.5

    def test_failure_result_fields(self):
        from lunabot_control.deposition_bridge import DepositionBridge
        r = DepositionBridge._result(
            False,
            Deposit.Result.REASON_ESTOP,
            "E-stop",
            2.5,
        )
        assert r.success is False
        assert r.reason_code == Deposit.Result.REASON_ESTOP
        assert r.residual_fill_fraction_estimate >= 0.5


class TestDepositPhases:
    """Verify feedback phase constants match action definition."""

    def test_phase_constants_exist(self):
        assert hasattr(Deposit.Feedback, "PHASE_PRECHECK")
        assert hasattr(Deposit.Feedback, "PHASE_OPENING")
        assert hasattr(Deposit.Feedback, "PHASE_RAISING")
        assert hasattr(Deposit.Feedback, "PHASE_DUMPING")
        assert hasattr(Deposit.Feedback, "PHASE_CLOSING")

    def test_phases_are_sequential(self):
        phases = [
            Deposit.Feedback.PHASE_PRECHECK,
            Deposit.Feedback.PHASE_OPENING,
            Deposit.Feedback.PHASE_RAISING,
            Deposit.Feedback.PHASE_DUMPING,
            Deposit.Feedback.PHASE_CLOSING,
        ]
        for i in range(len(phases) - 1):
            assert phases[i] < phases[i + 1]


class TestDepositHardwareAvailability:
    """Verify deposit goals fail closed when actuator hardware is unavailable."""

    def _make_bridge(self, *, dry_run=False):
        from lunabot_control.deposition_bridge import DepositionBridge

        bridge = object.__new__(DepositionBridge)
        bridge._estop_active = False
        bridge._motion_inhibited = False
        bridge._gpio_available = False
        bridge._dry_run = dry_run
        bridge._hardware_fault_reason = "GPIO unavailable"
        bridge.get_logger = MagicMock()
        return bridge

    def test_goal_rejected_when_gpio_missing_and_not_dry_run(self):
        bridge = self._make_bridge(dry_run=False)

        response = bridge._goal_cb(object())

        assert response == GoalResponse.REJECT
        bridge.get_logger().error.assert_called_once()

    def test_goal_accepted_when_gpio_missing_but_dry_run_explicit(self):
        bridge = self._make_bridge(dry_run=True)

        response = bridge._goal_cb(object())

        assert response == GoalResponse.ACCEPT

    def test_execute_aborts_when_hardware_unavailable(self):
        bridge = self._make_bridge(dry_run=False)
        goal_handle = MagicMock()

        result = bridge._execute_deposit(goal_handle)

        goal_handle.abort.assert_called_once()
        assert result.success is False
        assert result.reason_code == Deposit.Result.REASON_DRIVER_FAULT
        assert result.failure_reason == "GPIO unavailable"
