"""Unit tests for rover diagnostics status mapping."""

from std_msgs.msg import Bool

from lunabot_bringup.rover_diagnostics import (
    LEVEL_ERROR,
    LEVEL_OK,
    LEVEL_STALE,
    LEVEL_WARN,
    TopicSample,
    drivetrain_diagnostic,
    excavation_diagnostic,
    localisation_diagnostic,
    safety_diagnostic,
)
from lunabot_interfaces.msg import (
    DrivetrainStatus,
    ExcavationStatus,
    LocalisationStartZoneStatus,
)


def _sample(msg, received_at_s=10.0):
    return TopicSample(msg=msg, received_at_s=received_at_s)


def test_drivetrain_diagnostic_is_stale_without_status():
    status = drivetrain_diagnostic(
        TopicSample(), now_s=20.0, timeout_s=2.5
    )

    assert status.level == LEVEL_STALE
    assert "No fresh drivetrain" in status.message


def test_drivetrain_diagnostic_reports_fault_as_error():
    msg = DrivetrainStatus()
    msg.state = DrivetrainStatus.STATE_FAULT
    msg.fault_code = DrivetrainStatus.FAULT_CONTROLLER_OFFLINE
    msg.controller_online = [False, False]

    status = drivetrain_diagnostic(
        _sample(msg), now_s=10.1, timeout_s=2.5
    )

    assert status.level == LEVEL_ERROR


def test_drivetrain_diagnostic_reports_ready_as_ok():
    msg = DrivetrainStatus()
    msg.state = DrivetrainStatus.STATE_READY
    msg.fault_code = DrivetrainStatus.FAULT_NONE
    msg.controller_online = [True, True]

    status = drivetrain_diagnostic(
        _sample(msg), now_s=10.1, timeout_s=2.5
    )

    assert status.level == LEVEL_OK


def test_safety_diagnostic_reports_estop_as_error():
    estop = Bool()
    estop.data = True
    inhibit = Bool()
    inhibit.data = False

    status = safety_diagnostic(
        _sample(estop), _sample(inhibit), now_s=10.1, timeout_s=2.5
    )

    assert status.level == LEVEL_ERROR
    assert status.message == "E-stop active"


def test_safety_diagnostic_reports_clear_as_ok():
    estop = Bool()
    estop.data = False
    inhibit = Bool()
    inhibit.data = False

    status = safety_diagnostic(
        _sample(estop), _sample(inhibit), now_s=10.1, timeout_s=2.5
    )

    assert status.level == LEVEL_OK


def test_localisation_diagnostic_reports_not_ready_as_warn():
    msg = LocalisationStartZoneStatus()
    msg.ready = False
    msg.state = "searching"
    msg.reason_code = "no_tag"
    msg.reason_detail = "No tag lock"

    status = localisation_diagnostic(
        _sample(msg), now_s=10.1, timeout_s=2.5
    )

    assert status.level == LEVEL_WARN
    assert status.message == "No tag lock"


def test_excavation_diagnostic_reports_unhomed_as_warn():
    msg = ExcavationStatus()
    msg.state = ExcavationStatus.STATE_IDLE
    msg.fault_code = ExcavationStatus.FAULT_NONE
    msg.homed = False

    status = excavation_diagnostic(
        _sample(msg), now_s=10.1, timeout_s=2.5
    )

    assert status.level == LEVEL_WARN
    assert status.message == "Excavation not homed"


def test_excavation_diagnostic_reports_fault_as_error():
    msg = ExcavationStatus()
    msg.state = ExcavationStatus.STATE_FAULT
    msg.fault_code = ExcavationStatus.FAULT_DRIVER
    msg.homed = True

    status = excavation_diagnostic(
        _sample(msg), now_s=10.1, timeout_s=2.5
    )

    assert status.level == LEVEL_ERROR
