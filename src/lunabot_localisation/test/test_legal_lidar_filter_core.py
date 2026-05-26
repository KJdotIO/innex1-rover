"""Unit tests for legal LiDAR point-cloud filtering."""

from types import SimpleNamespace

import numpy as np
import pytest

from lunabot_localisation.legal_lidar_filter_core import (
    ArenaBounds,
    cloud_records,
    filter_cloud_to_legal_bounds,
    make_point_dtype,
    transform_points,
)

FLOAT32 = 7
UINT16 = 4
UINT32 = 6


def _field(name, offset, datatype, count=1):
    return SimpleNamespace(name=name, offset=offset, datatype=datatype, count=count)


def _cloud(points):
    fields = [
        _field("x", 0, FLOAT32),
        _field("y", 4, FLOAT32),
        _field("z", 8, FLOAT32),
        _field("intensity", 12, FLOAT32),
        _field("t", 16, UINT32),
        _field("ring", 20, UINT16),
    ]
    point_step = 24
    raw_data = bytearray(point_step * len(points))
    records = np.ndarray(
        len(points),
        dtype=make_point_dtype(fields, point_step, is_bigendian=False),
        buffer=raw_data,
    )
    for index, (x, y, z) in enumerate(points):
        records[index]["x"] = x
        records[index]["y"] = y
        records[index]["z"] = z
        records[index]["intensity"] = 100.0 + index
        records[index]["t"] = 1000 + index
        records[index]["ring"] = 42 + index

    return SimpleNamespace(
        height=1,
        width=len(points),
        fields=fields,
        is_bigendian=False,
        point_step=point_step,
        row_step=point_step * len(points),
        is_dense=False,
        data=bytes(raw_data),
    )


def test_filter_preserves_point_fields_and_rejects_wall_band():
    """Only legal points pass, with Ouster-style metadata still attached."""
    cloud = _cloud(
        [
            (0.0, 0.0, 0.2),
            (-0.9, 0.0, 0.2),
            (8.0, 0.0, 0.2),
            (np.nan, 0.0, 0.2),
        ]
    )
    bounds = ArenaBounds(
        min_x=-1.0,
        max_x=6.9,
        min_y=-3.3,
        max_y=1.1,
        wall_exclusion_margin_m=0.35,
    )

    result = filter_cloud_to_legal_bounds(
        cloud,
        bounds,
        translation_xyz=(0.0, 0.0, 0.0),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
    )

    assert result.raw_count == 4
    assert result.finite_count == 3
    assert result.kept_count == 1
    assert result.rejected_count == 2
    assert result.reject_ratio == pytest.approx(2.0 / 3.0)
    assert result.cloud.width == 1
    assert result.cloud.height == 1
    assert result.cloud.point_step == cloud.point_step
    assert [field.name for field in result.cloud.fields] == [
        "x",
        "y",
        "z",
        "intensity",
        "t",
        "ring",
    ]

    kept = cloud_records(result.cloud)
    assert kept["intensity"][0] == pytest.approx(100.0)
    assert kept["t"][0] == 1000
    assert kept["ring"][0] == 42


def test_filter_uses_mask_frame_transform_without_rewriting_output_frame_data():
    """The transform is a legal-membership gate, not an odometry-frame rewrite."""
    cloud = _cloud([(0.0, 0.0, 0.2)])
    bounds = ArenaBounds(min_x=0.0, max_x=2.0, min_y=0.0, max_y=2.0)

    result = filter_cloud_to_legal_bounds(
        cloud,
        bounds,
        translation_xyz=(0.5, 0.5, 0.0),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
    )

    kept = cloud_records(result.cloud)
    assert result.kept_count == 1
    assert kept["x"][0] == pytest.approx(0.0)
    assert kept["y"][0] == pytest.approx(0.0)


def test_filter_rejects_unknown_point_field_datatype():
    """Unsupported PointCloud2 datatypes fail loudly."""
    cloud = _cloud([(0.0, 0.0, 0.0)])
    cloud.fields[0].datatype = 99

    with pytest.raises(ValueError, match="Unsupported PointCloud2 datatype"):
        filter_cloud_to_legal_bounds(
            cloud,
            ArenaBounds(),
            translation_xyz=(0.0, 0.0, 0.0),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        )


def test_transform_points_applies_translation_and_yaw():
    """The core transform matches the filter's legal-mask geometry."""
    points = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    quaternion_xyzw = (0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5))

    transformed = transform_points(points, (2.0, -1.0, 0.25), quaternion_xyzw)

    np.testing.assert_allclose(transformed, [[2.0, 0.0, 0.25]], atol=1e-6)
