"""Unit tests for start-zone localisation readiness gating."""

from types import SimpleNamespace

from lunabot_bringup.localisation_readiness import is_localisation_ready


def _status(*, ready: bool, state: str, reason_code: str):
    return SimpleNamespace(ready=ready, state=state, reason_code=reason_code)


def test_localisation_ready_requires_fresh_ready_state():
    status = _status(
        ready=True,
        state="ready",
        reason_code="LOCALISATION_READY",
    )

    assert is_localisation_ready(
        status,
        last_update_ns=2_000_000_000,
        now_ns=3_000_000_000,
        freshness_ns=5_000_000_000,
    )


def test_localisation_ready_rejects_stale_status():
    status = _status(
        ready=True,
        state="ready",
        reason_code="LOCALISATION_READY",
    )

    assert not is_localisation_ready(
        status,
        last_update_ns=2_000_000_000,
        now_ns=8_000_000_000,
        freshness_ns=5_000_000_000,
    )


def test_localisation_ready_rejects_non_ready_reason():
    status = _status(
        ready=True,
        state="candidate_lock",
        reason_code="LOCALISATION_TAG_UNSTABLE",
    )

    assert not is_localisation_ready(
        status,
        last_update_ns=2_000_000_000,
        now_ns=3_000_000_000,
        freshness_ns=5_000_000_000,
    )
