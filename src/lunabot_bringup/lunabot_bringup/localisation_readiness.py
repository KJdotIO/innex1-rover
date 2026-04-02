"""Helpers for interpreting start-zone localisation readiness."""

READY_STATE = "ready"
READY_REASON = "LOCALISATION_READY"


def is_localisation_ready(
    status,
    last_update_ns: int | None,
    now_ns: int,
    freshness_ns: int,
) -> bool:
    """Return whether the latest status still counts as travel-ready."""
    del last_update_ns
    if status is None:
        return False

    status_ns = int(status.stamp.sec) * 1_000_000_000 + int(status.stamp.nanosec)
    if status_ns <= 0:
        return False

    if now_ns <= 0 or now_ns < status_ns:
        return False

    if (now_ns - status_ns) > freshness_ns:
        return False

    return (
        bool(status.ready)
        and status.state == READY_STATE
        and status.reason_code == READY_REASON
    )
