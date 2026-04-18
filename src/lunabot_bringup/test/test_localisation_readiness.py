"""Unit tests for start-zone localisation readiness gating."""

from types import SimpleNamespace

from lunabot_bringup.localisation_readiness import (
    GATE_NO_STATUS,
    GATE_STATE_NOT_READY,
    GATE_STATUS_STALE,
    get_readiness_failure_reason,
    is_localisation_ready,
)


def _status(*, ready: bool, state: str, reason_code: str, stamp_ns: int):
    return SimpleNamespace(
        ready=ready,
        state=state,
        reason_code=reason_code,
        stamp=SimpleNamespace(
            sec=stamp_ns // 1_000_000_000,
            nanosec=stamp_ns % 1_000_000_000,
        ),
    )


def test_localisation_ready_requires_fresh_ready_state():
    status = _status(
        ready=True,
        state="ready",
        reason_code="LOCALISATION_READY",
        stamp_ns=2_000_000_000,
    )

    assert is_localisation_ready(
        status,
        last_update_ns=None,
        now_ns=3_000_000_000,
        freshness_ns=5_000_000_000,
    )


def test_localisation_ready_rejects_stale_status():
    status = _status(
        ready=True,
        state="ready",
        reason_code="LOCALISATION_READY",
        stamp_ns=2_000_000_000,
    )

    assert not is_localisation_ready(
        status,
        last_update_ns=None,
        now_ns=8_000_000_000,
        freshness_ns=5_000_000_000,
    )


def test_localisation_ready_rejects_backwards_clock():
    status = _status(
        ready=True,
        state="ready",
        reason_code="LOCALISATION_READY",
        stamp_ns=2_000_000_000,
    )

    assert not is_localisation_ready(
        status,
        last_update_ns=None,
        now_ns=1_500_000_000,
        freshness_ns=5_000_000_000,
    )


def test_localisation_ready_rejects_non_ready_reason():
    status = _status(
        ready=True,
        state="candidate_lock",
        reason_code="LOCALISATION_TAG_UNSTABLE",
        stamp_ns=2_000_000_000,
    )

    assert not is_localisation_ready(
        status,
        last_update_ns=None,
        now_ns=3_000_000_000,
        freshness_ns=5_000_000_000,
    )


def test_localisation_ready_rejects_missing_source_stamp():
    status = _status(
        ready=True,
        state="ready",
        reason_code="LOCALISATION_READY",
        stamp_ns=0,
    )

    assert not is_localisation_ready(
        status,
        last_update_ns=None,
        now_ns=3_000_000_000,
        freshness_ns=5_000_000_000,
    )


# ---------------------------------------------------------------------------
# get_readiness_failure_reason tests
# ---------------------------------------------------------------------------


def test_failure_reason_none_status_returns_gate_no_status():
    """None status yields GATE_NO_STATUS."""
    reason = get_readiness_failure_reason(None, now_ns=3_000_000_000, freshness_ns=5_000_000_000)
    assert reason == GATE_NO_STATUS


def test_failure_reason_zero_stamp_returns_gate_no_status():
    """A zero timestamp (uninitialised) yields GATE_NO_STATUS."""
    status = _status(
        ready=True,
        state="ready",
        reason_code="LOCALISATION_READY",
        stamp_ns=0,
    )
    reason = get_readiness_failure_reason(
        status, now_ns=3_000_000_000, freshness_ns=5_000_000_000
    )
    assert reason == GATE_NO_STATUS


def test_failure_reason_stale_status_returns_gate_status_stale():
    """A status older than freshness_ns yields GATE_STATUS_STALE."""
    status = _status(
        ready=True,
        state="ready",
        reason_code="LOCALISATION_READY",
        stamp_ns=2_000_000_000,
    )
    reason = get_readiness_failure_reason(
        status, now_ns=8_000_000_000, freshness_ns=5_000_000_000
    )
    assert reason == GATE_STATUS_STALE


def test_failure_reason_backwards_clock_returns_gate_status_stale():
    """now_ns behind stamp_ns (backwards/zero clock) yields GATE_STATUS_STALE."""
    status = _status(
        ready=True,
        state="ready",
        reason_code="LOCALISATION_READY",
        stamp_ns=2_000_000_000,
    )
    reason = get_readiness_failure_reason(
        status, now_ns=1_500_000_000, freshness_ns=5_000_000_000
    )
    assert reason == GATE_STATUS_STALE


def test_failure_reason_wrong_state_returns_gate_state_not_ready():
    """Wrong state field yields GATE_STATE_NOT_READY."""
    status = _status(
        ready=True,
        state="candidate_lock",
        reason_code="LOCALISATION_READY",
        stamp_ns=2_000_000_000,
    )
    reason = get_readiness_failure_reason(
        status, now_ns=3_000_000_000, freshness_ns=5_000_000_000
    )
    assert reason == GATE_STATE_NOT_READY


def test_failure_reason_wrong_reason_code_returns_gate_state_not_ready():
    """Wrong reason_code field yields GATE_STATE_NOT_READY."""
    status = _status(
        ready=True,
        state="ready",
        reason_code="LOCALISATION_TAG_UNSTABLE",
        stamp_ns=2_000_000_000,
    )
    reason = get_readiness_failure_reason(
        status, now_ns=3_000_000_000, freshness_ns=5_000_000_000
    )
    assert reason == GATE_STATE_NOT_READY


def test_failure_reason_ready_status_returns_none():
    """A fully ready status yields None (no failure)."""
    status = _status(
        ready=True,
        state="ready",
        reason_code="LOCALISATION_READY",
        stamp_ns=2_000_000_000,
    )
    reason = get_readiness_failure_reason(
        status, now_ns=3_000_000_000, freshness_ns=5_000_000_000
    )
    assert reason is None
