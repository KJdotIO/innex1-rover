from pathlib import Path

from lunabot_bringup.power_telemetry import (
    PowerProfile,
    build_power_telemetry,
    classify_power_state,
    load_power_profiles,
)

from lunabot_interfaces.msg import PowerTelemetry


def _profile():
    return PowerProfile(
        name="lipo_6s",
        cells=6,
        warning_voltage_v=21.0,
        critical_voltage_v=19.8,
    )


def test_classify_power_state_reports_nominal_voltage_ok():
    assert classify_power_state(22.2, _profile()) == PowerTelemetry.STATE_OK


def test_classify_power_state_reports_warning_threshold():
    assert (
        classify_power_state(21.0, _profile())
        == PowerTelemetry.STATE_LOW_WARNING
    )


def test_classify_power_state_reports_critical_threshold():
    assert (
        classify_power_state(19.8, _profile())
        == PowerTelemetry.STATE_LOW_CRITICAL
    )


def test_classify_power_state_reports_unavailable_without_voltage():
    assert (
        classify_power_state(None, _profile())
        == PowerTelemetry.STATE_UNAVAILABLE
    )


def test_build_power_telemetry_sets_threshold_flags():
    msg = build_power_telemetry(
        voltage_v=19.7,
        current_a=4.0,
        energy_wh=12.5,
        profile=_profile(),
        source="manual",
    )

    assert msg.state == PowerTelemetry.STATE_LOW_CRITICAL
    assert msg.low_voltage_warning is True
    assert msg.low_voltage_critical is True
    assert msg.warning_voltage_v == 21.0
    assert msg.critical_voltage_v == 19.8


def test_load_power_profiles_reads_config():
    path = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "power_profiles.yaml"
    )

    profiles = load_power_profiles(path)

    assert profiles["lipo_4s"].warning_voltage_v == 14.0
    assert profiles["lipo_6s"].critical_voltage_v == 19.8
