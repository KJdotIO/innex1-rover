# Copyright 2026 University of Leicester
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Unit tests for map-frame interior cropping (wall exclusion)."""

import numpy as np
import pytest

from lunabot_perception.wall_exclusion_geometry import (
    apply_rigid_transform,
    compute_interior_bounds,
    interior_xy_mask,
)


def test_interior_xy_mask_keeps_center_drops_outside():
    bounds = (0.0, 10.0, -2.0, 2.0)
    x = np.array([5.0, -1.0, 10.01, 0.0])
    y = np.array([0.0, 0.0, 0.0, 2.0])
    mask = interior_xy_mask(x, y, bounds)
    assert mask.tolist() == [True, False, False, True]


def test_interior_xy_mask_empty_arrays():
    bounds = (0.0, 1.0, 0.0, 1.0)
    x = np.array([], dtype=np.float64)
    y = np.array([], dtype=np.float64)
    mask = interior_xy_mask(x, y, bounds)
    assert mask.size == 0


def test_compute_interior_bounds_rejects_empty_after_inset():
    with pytest.raises(ValueError, match="After inset_margin_m"):
        compute_interior_bounds(
            -1.0,
            6.9,
            -3.3,
            1.1,
            50.0,
        )


def test_apply_rigid_transform_rotates_and_translates():
    points = np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 0.5]], dtype=np.float64)
    # +90 deg yaw about Z, then translate by (1, 2, 3).
    quat = (0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5))
    transformed = apply_rigid_transform(points, (1.0, 2.0, 3.0), quat)
    expected = np.array([[1.0, 3.0, 3.0], [-1.0, 2.0, 3.5]], dtype=np.float64)
    assert np.allclose(transformed, expected)


def test_apply_rigid_transform_rejects_zero_quaternion():
    points = np.zeros((1, 3), dtype=np.float64)
    with pytest.raises(ValueError, match="quaternion must be non-zero"):
        apply_rigid_transform(points, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))
