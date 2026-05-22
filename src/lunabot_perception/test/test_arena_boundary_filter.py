"""Unit tests for arena wall-exclusion filtering."""

import numpy as np
import pytest

from lunabot_perception.arena_boundary_filter import (
    ArenaBounds,
    filter_points_to_arena,
    transform_points,
)


def test_arena_bounds_reject_invalid_margin():
    """The exclusion margin cannot remove the whole usable arena."""
    with pytest.raises(ValueError, match="removes all legal arena height"):
        ArenaBounds(
            min_x=0.0, max_x=10.0, min_y=0.0, max_y=1.0, wall_exclusion_margin_m=0.5
        )


def test_filter_rejects_wall_band_and_outside_points():
    """Only points in the legal interior pass through."""
    bounds = ArenaBounds(
        min_x=-1.0,
        max_x=6.9,
        min_y=-3.3,
        max_y=1.1,
        wall_exclusion_margin_m=0.35,
    )
    points = np.array(
        [
            [0.0, 0.0, 0.2],  # legal interior
            [-0.9, 0.0, 0.2],  # west wall band
            [6.8, 0.0, 0.2],  # east wall band
            [2.0, -3.2, 0.2],  # south wall band
            [2.0, 1.0, 0.2],  # north wall band
            [9.0, 0.0, 0.2],  # outside arena
            [np.nan, 0.0, 0.2],  # non-finite point
        ],
        dtype=np.float32,
    )

    result = filter_points_to_arena(points, bounds)

    assert result.input_count == 6
    assert result.kept_count == 1
    assert result.rejected_count == 5
    assert result.reject_ratio == pytest.approx(5.0 / 6.0)
    np.testing.assert_allclose(result.points, [[0.0, 0.0, 0.2]])


def test_filter_accepts_points_on_legal_boundary():
    """The configured margin is inclusive for points exactly on the legal edge."""
    bounds = ArenaBounds(
        min_x=0.0, max_x=4.0, min_y=0.0, max_y=4.0, wall_exclusion_margin_m=0.5
    )
    points = np.array(
        [
            [0.5, 2.0, 0.0],
            [3.5, 2.0, 0.0],
            [2.0, 0.5, 0.0],
            [2.0, 3.5, 0.0],
        ],
        dtype=np.float32,
    )

    result = filter_points_to_arena(points, bounds)

    assert result.kept_count == 4


def test_transform_points_applies_translation_and_yaw():
    """Point transforms match the target-frame geometry used before filtering."""
    points = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    # 90 degrees around z: (1, 0, 0) -> (0, 1, 0), then translate.
    quaternion_xyzw = (0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5))

    transformed = transform_points(points, (2.0, -1.0, 0.25), quaternion_xyzw)

    np.testing.assert_allclose(transformed, [[2.0, 0.0, 0.25]], atol=1e-6)
