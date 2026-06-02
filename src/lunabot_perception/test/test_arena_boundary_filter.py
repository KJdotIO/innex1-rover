"""Unit tests for arena wall-exclusion filtering."""

import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField

from lunabot_perception.arena_boundary_filter import (
    ArenaBounds,
    cloud_xyz,
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


def test_cloud_xyz_accepts_mixed_os1_style_fields():
    """OS1 clouds mix float coordinates with integer ring/time fields."""
    dtype = np.dtype(
        {
            "names": ["x", "y", "z", "intensity", "t", "ring"],
            "formats": [
                np.float32,
                np.float32,
                np.float32,
                np.float32,
                np.uint32,
                np.uint16,
            ],
            "offsets": [0, 4, 8, 16, 20, 24],
            "itemsize": 32,
        }
    )
    records = np.zeros(2, dtype=dtype)
    records["x"] = [1.0, 2.0]
    records["y"] = [3.0, 4.0]
    records["z"] = [5.0, 6.0]
    records["t"] = [10, 20]
    records["ring"] = [1, 2]

    cloud = PointCloud2()
    cloud.height = 1
    cloud.width = 2
    cloud.point_step = 32
    cloud.row_step = 64
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name="t", offset=20, datatype=PointField.UINT32, count=1),
        PointField(name="ring", offset=24, datatype=PointField.UINT16, count=1),
    ]
    cloud.data = records.tobytes()

    np.testing.assert_allclose(cloud_xyz(cloud), [[1, 3, 5], [2, 4, 6]])
