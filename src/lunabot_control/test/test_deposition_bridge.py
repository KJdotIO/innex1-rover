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
