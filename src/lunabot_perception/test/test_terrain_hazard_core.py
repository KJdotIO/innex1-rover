"""Unit tests for terrain hazard grid helpers."""

import numpy as np

from lunabot_perception.terrain_hazard_core import GridSpec
from lunabot_perception.terrain_hazard_core import build_elevation_grid
from lunabot_perception.terrain_hazard_core import detect_drop_cells
from lunabot_perception.terrain_hazard_core import filter_clusters
from lunabot_perception.terrain_hazard_core import occupancy_values
from lunabot_perception.terrain_hazard_core import pad_hazards


def test_build_elevation_grid_tracks_min_max_and_counts():
    grid = GridSpec(min_x=0.0, max_x=0.4, min_y=0.0, max_y=0.4, resolution=0.2)
    points = np.array(
        [
            [0.05, 0.05, 0.10],
            [0.08, 0.06, 0.20],
            [0.25, 0.25, -0.30],
        ],
        dtype=np.float32,
    )

    min_height, max_height, counts = build_elevation_grid(points, grid)

    assert counts[0, 0] == 2
    assert np.isclose(min_height[0, 0], 0.10)
    assert np.isclose(max_height[0, 0], 0.20)
    assert counts[1, 1] == 1
    assert np.isclose(min_height[1, 1], -0.30)


def test_detect_drop_cells_marks_supported_crater_cell():
    grid = GridSpec(min_x=0.0, max_x=0.6, min_y=0.0, max_y=0.6, resolution=0.2)
    counts = np.full((grid.height, grid.width), 4, dtype=np.int32)
    min_height = np.zeros((grid.height, grid.width), dtype=np.float32)
    max_height = np.zeros((grid.height, grid.width), dtype=np.float32)
    min_height[1, 1] = -0.25
    max_height[1, 1] = -0.25

    hazards, unknown = detect_drop_cells(
        min_height=min_height,
        max_height=max_height,
        counts=counts,
        min_points_per_cell=3,
        neighborhood_radius_cells=1,
        min_neighbor_cells=4,
        drop_threshold=0.14,
        roughness_threshold=0.10,
        edge_margin_cells=0,
        max_detection_x=1.0,
        grid=grid,
    )

    assert hazards[1, 1]
    assert not unknown[1, 1]


def test_detect_drop_cells_marks_sparse_cells_unknown():
    grid = GridSpec(min_x=0.0, max_x=0.4, min_y=0.0, max_y=0.4, resolution=0.2)
    counts = np.zeros((grid.height, grid.width), dtype=np.int32)
    min_height = np.zeros((grid.height, grid.width), dtype=np.float32)
    max_height = np.zeros((grid.height, grid.width), dtype=np.float32)

    hazards, unknown = detect_drop_cells(
        min_height=min_height,
        max_height=max_height,
        counts=counts,
        min_points_per_cell=3,
        neighborhood_radius_cells=1,
        min_neighbor_cells=4,
        drop_threshold=0.14,
        roughness_threshold=0.10,
        edge_margin_cells=0,
        max_detection_x=1.0,
        grid=grid,
    )

    assert not hazards.any()
    assert unknown.all()


def test_detect_drop_cells_keeps_edge_margin_unknown():
    grid = GridSpec(min_x=0.0, max_x=0.6, min_y=0.0, max_y=0.6, resolution=0.2)
    counts = np.full((grid.height, grid.width), 4, dtype=np.int32)
    min_height = np.zeros((grid.height, grid.width), dtype=np.float32)
    max_height = np.zeros((grid.height, grid.width), dtype=np.float32)

    _, unknown = detect_drop_cells(
        min_height=min_height,
        max_height=max_height,
        counts=counts,
        min_points_per_cell=3,
        neighborhood_radius_cells=1,
        min_neighbor_cells=4,
        drop_threshold=0.14,
        roughness_threshold=0.10,
        edge_margin_cells=1,
        max_detection_x=1.0,
        grid=grid,
    )

    assert unknown[0, 0]
    assert unknown[0, 1]
    assert unknown[1, 0]


def test_filter_clusters_removes_tiny_groups():
    hazards = np.zeros((4, 4), dtype=bool)
    hazards[0, 0] = True
    hazards[2, 2] = True
    hazards[2, 3] = True

    filtered = filter_clusters(hazards, min_cluster_cells=2)

    assert not filtered[0, 0]
    assert filtered[2, 2]
    assert filtered[2, 3]


def test_pad_hazards_dilates_cells():
    hazards = np.zeros((3, 3), dtype=bool)
    hazards[1, 1] = True

    padded = pad_hazards(hazards, padding_cells=1)

    assert padded.all()


def test_occupancy_values_render_unknown_and_hazard_cells():
    hazards = np.array([[False, True], [False, False]], dtype=bool)
    unknown = np.array([[True, False], [False, True]], dtype=bool)

    rendered = occupancy_values(hazards=hazards, unknown=unknown)

    assert rendered.tolist() == [[-1, 100], [0, -1]]
