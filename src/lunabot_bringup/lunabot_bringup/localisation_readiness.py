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
    if status is None or last_update_ns is None:
        return False

    if (now_ns - last_update_ns) > freshness_ns:
        return False

    return (
        bool(status.ready)
        and status.state == READY_STATE
        and status.reason_code == READY_REASON
    )
