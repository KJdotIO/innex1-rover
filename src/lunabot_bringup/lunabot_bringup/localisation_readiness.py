"""Helpers for interpreting start-zone localisation readiness."""

READY_STATE = "ready"
READY_REASON = "LOCALISATION_READY"

GATE_NO_STATUS = "GATE_NO_STATUS"
GATE_STATUS_STALE = "GATE_STATUS_STALE"
GATE_STATE_NOT_READY = "GATE_STATE_NOT_READY"


def get_readiness_failure_reason(
    status,
    now_ns: int,
    freshness_ns: int,
) -> str | None:
    """Return a GATE_* reason code for the first localisation failure, or None."""
    if status is None:
        return GATE_NO_STATUS

    status_ns = int(status.stamp.sec) * 1_000_000_000 + int(status.stamp.nanosec)
    if status_ns <= 0:
        return GATE_NO_STATUS

    if now_ns <= 0 or now_ns < status_ns:
        return GATE_STATUS_STALE

    if (now_ns - status_ns) > freshness_ns:
        return GATE_STATUS_STALE

    if (
        not bool(status.ready)
        or status.state != READY_STATE
        or status.reason_code != READY_REASON
    ):
        return GATE_STATE_NOT_READY

    return None


def is_localisation_ready(
    status,
    last_update_ns: int | None,
    now_ns: int,
    freshness_ns: int,
) -> bool:
    """Return whether the latest status still counts as travel-ready."""
    del last_update_ns
    return get_readiness_failure_reason(status, now_ns, freshness_ns) is None
