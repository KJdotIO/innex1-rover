"""Helpers for phase-scoped preflight checks."""

from __future__ import annotations

from copy import deepcopy
from typing import Any

CHECK_SECTIONS = (
    "required_topics",
    "required_tf_links",
    "required_actions",
    "required_nodes",
)
PHASE_FULL = "full"
PHASE_LAUNCH = "launch"
PHASE_RUNTIME = "runtime"
VALID_PHASES = {PHASE_FULL, PHASE_LAUNCH, PHASE_RUNTIME}


def validate_phase(phase: str) -> str:
    """Validate and normalise a requested preflight phase."""
    if phase not in VALID_PHASES:
        valid = ", ".join(sorted(VALID_PHASES))
        raise ValueError(
            f"Unsupported preflight phase '{phase}'. Expected one of: {valid}."
        )
    return phase


def phase_matches(item: dict[str, Any], phase: str) -> bool:
    """Return True when a check item should run in the given phase."""
    phase = validate_phase(phase)
    if phase == PHASE_FULL:
        return True

    configured_phases = item.get("phases")
    if configured_phases is None:
        return True
    if not isinstance(configured_phases, list):
        raise ValueError(
            f"Invalid phases value for check item {item!r}; expected a list."
        )
    return phase in configured_phases


def filter_preflight_config(config: dict[str, Any], phase: str) -> dict[str, Any]:
    """Return a copy of the config filtered to the requested phase."""
    phase = validate_phase(phase)
    if phase == PHASE_FULL:
        return deepcopy(config)

    filtered = deepcopy(config)
    preflight = filtered.setdefault("preflight", {})
    for section in CHECK_SECTIONS:
        entries = preflight.get(section, [])
        if not isinstance(entries, list):
            continue
        preflight[section] = [
            entry for entry in entries if phase_matches(entry, phase)
        ]
    return filtered
