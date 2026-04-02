"""Unit tests for start-zone localisation readiness gating."""

from types import SimpleNamespace

from lunabot_bringup.localisation_readiness import is_localisation_ready


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
