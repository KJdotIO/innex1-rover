"""Field-preserving arena filtering for LiDAR localisation clouds."""

from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np


_DATATYPE_TO_DTYPE = {
    1: np.int8,
    2: np.uint8,
    3: np.int16,
    4: np.uint16,
    5: np.int32,
    6: np.uint32,
    7: np.float32,
    8: np.float64,
}


@dataclass(frozen=True)
class ArenaBounds:
    """Interior arena bounds plus a wall-exclusion margin."""

    min_x: float = -1.0
    max_x: float = 6.9
    min_y: float = -3.3
    max_y: float = 1.1
    wall_exclusion_margin_m: float = 0.35

    def __post_init__(self) -> None:
        """Validate that the legal area remains non-empty."""
        if self.min_x >= self.max_x:
            raise ValueError("arena_min_x must be less than arena_max_x")
        if self.min_y >= self.max_y:
            raise ValueError("arena_min_y must be less than arena_max_y")
        if self.wall_exclusion_margin_m < 0.0:
            raise ValueError("wall_exclusion_margin_m must be >= 0")
        if 2.0 * self.wall_exclusion_margin_m >= (self.max_x - self.min_x):
            raise ValueError("wall_exclusion_margin_m removes all legal arena width")
        if 2.0 * self.wall_exclusion_margin_m >= (self.max_y - self.min_y):
            raise ValueError("wall_exclusion_margin_m removes all legal arena height")

    @property
    def legal_min_x(self) -> float:
        """Lowest x coordinate allowed through the filter."""
        return self.min_x + self.wall_exclusion_margin_m

    @property
    def legal_max_x(self) -> float:
        """Highest x coordinate allowed through the filter."""
        return self.max_x - self.wall_exclusion_margin_m

    @property
    def legal_min_y(self) -> float:
        """Lowest y coordinate allowed through the filter."""
        return self.min_y + self.wall_exclusion_margin_m

    @property
    def legal_max_y(self) -> float:
        """Highest y coordinate allowed through the filter."""
        return self.max_y - self.wall_exclusion_margin_m


@dataclass(frozen=True)
class LegalLidarFilterResult:
    """Filtered cloud bytes and counts for localisation evidence."""

    cloud: SimpleNamespace
    raw_count: int
    finite_count: int
    kept_count: int
    rejected_count: int

    @property
    def reject_ratio(self) -> float:
        """Share of finite input points rejected by the arena filter."""
        if self.finite_count == 0:
            return 0.0
        return self.rejected_count / self.finite_count


def transform_points(
    points: np.ndarray,
    translation_xyz: tuple[float, float, float],
    quaternion_xyzw: tuple[float, float, float, float],
) -> np.ndarray:
    """Apply a rigid transform to an Nx3 point array."""
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must be an Nx3 array")
    x, y, z, w = quaternion_xyzw
    norm = np.sqrt((x * x) + (y * y) + (z * z) + (w * w))
    if norm == 0.0:
        raise ValueError("quaternion must be non-zero")
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    rotation = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    translation = np.array(translation_xyz, dtype=np.float64)
    return (rotation @ points.T).T + translation


def make_point_dtype(fields, point_step: int, is_bigendian: bool) -> np.dtype:
    """Build a structured dtype matching a PointCloud2 point layout."""
    names = []
    formats = []
    offsets = []
    endian = ">" if is_bigendian else "<"
    for field in fields:
        dtype = _DATATYPE_TO_DTYPE.get(field.datatype)
        if dtype is None:
            raise ValueError(f"Unsupported PointCloud2 datatype {field.datatype}")
        names.append(field.name)
        base_dtype = np.dtype(dtype).newbyteorder(endian)
        count = int(getattr(field, "count", 1))
        formats.append(base_dtype if count == 1 else (base_dtype, count))
        offsets.append(int(field.offset))
    return np.dtype(
        {
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": int(point_step),
        }
    )


def cloud_records(cloud) -> np.ndarray:
    """Return a contiguous structured array of cloud points without row padding."""
    width = int(cloud.width)
    height = int(cloud.height)
    point_step = int(cloud.point_step)
    row_step = int(cloud.row_step)
    dtype = make_point_dtype(cloud.fields, point_step, bool(cloud.is_bigendian))
    if width == 0 or height == 0:
        return np.empty(0, dtype=dtype)

    data = memoryview(cloud.data)
    rows = [
        np.frombuffer(
            data[row * row_step:row * row_step + width * point_step],
            dtype=dtype,
            count=width,
        )
        for row in range(height)
    ]
    return np.concatenate(rows).copy()


def selected_point_bytes(cloud, keep_mask: np.ndarray) -> bytes:
    """Copy selected point records from the original PointCloud2 byte layout."""
    width = int(cloud.width)
    point_step = int(cloud.point_step)
    row_step = int(cloud.row_step)
    data = memoryview(cloud.data)
    selected = bytearray(point_step * int(np.count_nonzero(keep_mask)))
    write_offset = 0
    for flat_index in np.flatnonzero(keep_mask):
        row = int(flat_index) // width
        column = int(flat_index) % width
        read_offset = row * row_step + column * point_step
        selected[write_offset:write_offset + point_step] = data[
            read_offset:read_offset + point_step
        ]
        write_offset += point_step
    return bytes(selected)


def cloud_xyz(cloud) -> np.ndarray:
    """Extract XYZ coordinates from a PointCloud2-like object."""
    records = cloud_records(cloud)
    missing = {"x", "y", "z"} - set(records.dtype.names or ())
    if missing:
        raise ValueError(f"PointCloud2 is missing required fields: {sorted(missing)}")
    return np.column_stack(
        [
            np.asarray(records["x"], dtype=np.float64).reshape(-1),
            np.asarray(records["y"], dtype=np.float64).reshape(-1),
            np.asarray(records["z"], dtype=np.float64).reshape(-1),
        ]
    )


def legal_point_mask(points_in_mask_frame: np.ndarray, bounds: ArenaBounds) -> np.ndarray:
    """Return a mask for finite points inside the legal arena interior."""
    if points_in_mask_frame.ndim != 2 or points_in_mask_frame.shape[1] != 3:
        raise ValueError("points must be an Nx3 array")
    return (
        np.all(np.isfinite(points_in_mask_frame), axis=1)
        & (points_in_mask_frame[:, 0] >= bounds.legal_min_x)
        & (points_in_mask_frame[:, 0] <= bounds.legal_max_x)
        & (points_in_mask_frame[:, 1] >= bounds.legal_min_y)
        & (points_in_mask_frame[:, 1] <= bounds.legal_max_y)
    )


def filter_cloud_to_legal_bounds(
    cloud,
    bounds: ArenaBounds,
    translation_xyz: tuple[float, float, float],
    quaternion_xyzw: tuple[float, float, float, float],
) -> LegalLidarFilterResult:
    """Filter a PointCloud2-like cloud while preserving its original fields."""
    records = cloud_records(cloud)
    xyz = np.column_stack(
        [
            np.asarray(records["x"], dtype=np.float64).reshape(-1),
            np.asarray(records["y"], dtype=np.float64).reshape(-1),
            np.asarray(records["z"], dtype=np.float64).reshape(-1),
        ]
    )
    finite_mask = np.all(np.isfinite(xyz), axis=1)
    transformed = transform_points(xyz, translation_xyz, quaternion_xyzw)
    keep_mask = legal_point_mask(transformed, bounds)
    kept_count = int(np.count_nonzero(keep_mask))

    filtered_cloud = SimpleNamespace(
        fields=list(cloud.fields),
        is_bigendian=bool(cloud.is_bigendian),
        point_step=int(cloud.point_step),
        row_step=int(cloud.point_step) * kept_count,
        width=kept_count,
        height=1,
        is_dense=True,
        data=selected_point_bytes(cloud, keep_mask),
    )
    finite_count = int(np.count_nonzero(finite_mask))
    return LegalLidarFilterResult(
        cloud=filtered_cloud,
        raw_count=int(records.shape[0]),
        finite_count=finite_count,
        kept_count=kept_count,
        rejected_count=finite_count - kept_count,
    )
