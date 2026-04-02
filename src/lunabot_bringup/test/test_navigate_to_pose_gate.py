"""Unit tests for NavigateToPose crater readiness gating."""

from lunabot_bringup.navigate_to_pose_gate import _hazard_readiness_is_fresh


def test_hazard_readiness_requires_detector_ready_signal():
    assert not _hazard_readiness_is_fresh(
        ready_value=False,
        ready_received_ns=5_000_000_000,
        grid_stamp_ns=5_000_000_000,
        now_ns=5_500_000_000,
        timeout_ns=2_000_000_000,
    )


def test_hazard_readiness_accepts_fresh_ready_signal_and_grid():
    assert _hazard_readiness_is_fresh(
        ready_value=True,
        ready_received_ns=5_000_000_000,
        grid_stamp_ns=4_900_000_000,
        now_ns=5_500_000_000,
        timeout_ns=2_000_000_000,
    )


def test_hazard_readiness_rejects_stale_grid_even_if_ready_signal_is_fresh():
    assert not _hazard_readiness_is_fresh(
        ready_value=True,
        ready_received_ns=5_500_000_000,
        grid_stamp_ns=2_000_000_000,
        now_ns=5_600_000_000,
        timeout_ns=2_000_000_000,
    )
