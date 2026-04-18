# Copyright 2026 University of Leicester
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Pure geometry for map-frame arena interior cropping (no ROS imports)."""

from __future__ import annotations

import numpy as np


def apply_rigid_transform(
    points: np.ndarray,
    translation_xyz: tuple[float, float, float],
    quaternion_xyzw: tuple[float, float, float, float],
) -> np.ndarray:
    """Apply a rigid transform to Nx3 XYZ points."""
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")

    qx, qy, qz, qw = quaternion_xyzw
    norm = float(np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw))
    if norm == 0.0:
        raise ValueError("quaternion must be non-zero")
    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    rotation = np.array(
        [
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qz * qw),
                2.0 * (qx * qz + qy * qw),
            ],
            [
                2.0 * (qx * qy + qz * qw),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qx * qw),
            ],
            [
                2.0 * (qx * qz - qy * qw),
                2.0 * (qy * qz + qx * qw),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ],
        ],
        dtype=np.float64,
    )
    translation = np.asarray(translation_xyz, dtype=np.float64)
    return points @ rotation.T + translation


def interior_xy_mask(
    x: np.ndarray,
    y: np.ndarray,
    bounds: tuple[float, float, float, float],
) -> np.ndarray:
    """Return a boolean mask for points inside the axis-aligned interior rectangle."""
    min_x, max_x, min_y, max_y = bounds
    return (x >= min_x) & (x <= max_x) & (y >= min_y) & (y <= max_y)


def compute_interior_bounds(
    arena_min_x: float,
    arena_max_x: float,
    arena_min_y: float,
    arena_max_y: float,
    inset_margin_m: float,
) -> tuple[float, float, float, float]:
    """Return (min_x, max_x, min_y, max_y) for the inset interior used for filtering."""
    if inset_margin_m < 0.0:
        raise ValueError("inset_margin_m must be non-negative")
    if arena_min_x >= arena_max_x or arena_min_y >= arena_max_y:
        raise ValueError("arena min/max must satisfy min < max for both axes")
    inner_min_x = arena_min_x + inset_margin_m
    inner_max_x = arena_max_x - inset_margin_m
    inner_min_y = arena_min_y + inset_margin_m
    inner_max_y = arena_max_y - inset_margin_m
    if inner_min_x >= inner_max_x or inner_min_y >= inner_max_y:
        raise ValueError(
            "After inset_margin_m, arena interior is empty; reduce inset or fix bounds.",
        )
    return (inner_min_x, inner_max_x, inner_min_y, inner_max_y)
