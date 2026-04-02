"""Pure helpers for launch-gated bringup."""

from __future__ import annotations


def select_preflight_config_path(
    lidar_costmap_phase: bool,
    default_config_path: str,
    lidar_debug_config_path: str,
) -> str:
    """Return the preflight config path for the requested launch mode."""
    if lidar_costmap_phase:
        return lidar_debug_config_path
    return default_config_path
