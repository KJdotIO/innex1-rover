"""Core helpers for local terrain hazard extraction."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class GridSpec:
    """Describe a rolling local grid in front of the rover."""

    min_x: float
    max_x: float
    min_y: float
    max_y: float
    resolution: float

    @property
    def width(self) -> int:
        """Return the grid width in cells."""
        return int(round((self.max_x - self.min_x) / self.resolution))

    @property
    def height(self) -> int:
        """Return the grid height in cells."""
        return int(round((self.max_y - self.min_y) / self.resolution))


def build_elevation_grid(
    points: np.ndarray,
    grid: GridSpec,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Project local terrain points into per-cell height statistics."""
    min_height = np.full((grid.height, grid.width), np.inf, dtype=np.float32)
    max_height = np.full((grid.height, grid.width), -np.inf, dtype=np.float32)
    counts = np.zeros((grid.height, grid.width), dtype=np.int32)

    x_indices = np.floor((points[:, 0] - grid.min_x) / grid.resolution).astype(int)
    y_indices = np.floor((points[:, 1] - grid.min_y) / grid.resolution).astype(int)
    valid = (
        (x_indices >= 0)
        & (x_indices < grid.width)
        & (y_indices >= 0)
        & (y_indices < grid.height)
    )

    for x_idx, y_idx, z_val in zip(
        x_indices[valid], y_indices[valid], points[valid, 2], strict=False
    ):
        counts[y_idx, x_idx] += 1
        if z_val < min_height[y_idx, x_idx]:
            min_height[y_idx, x_idx] = z_val
        if z_val > max_height[y_idx, x_idx]:
            max_height[y_idx, x_idx] = z_val

    return min_height, max_height, counts


def detect_drop_cells(
    *,
    min_height: np.ndarray,
    max_height: np.ndarray,
    counts: np.ndarray,
    min_points_per_cell: int,
    neighborhood_radius_cells: int,
    min_neighbor_cells: int,
    drop_threshold: float,
    roughness_threshold: float,
    edge_margin_cells: int,
    max_detection_x: float,
    grid: GridSpec,
) -> tuple[np.ndarray, np.ndarray]:
    """Classify cells into hazard or unknown using local terrain support."""
    hazards = np.zeros((grid.height, grid.width), dtype=bool)
    supported = counts >= min_points_per_cell
    unknown = ~supported

    for y_idx in range(edge_margin_cells, grid.height - edge_margin_cells):
        for x_idx in range(edge_margin_cells, grid.width - edge_margin_cells):
            x_center = grid.min_x + ((x_idx + 0.5) * grid.resolution)
            if x_center > max_detection_x:
                unknown[y_idx, x_idx] = True
                continue

            if not supported[y_idx, x_idx]:
                unknown[y_idx, x_idx] = True
                continue

            min_y = max(0, y_idx - neighborhood_radius_cells)
            max_y = min(grid.height, y_idx + neighborhood_radius_cells + 1)
            min_x = max(0, x_idx - neighborhood_radius_cells)
            max_x = min(grid.width, x_idx + neighborhood_radius_cells + 1)

            neighborhood_support = supported[min_y:max_y, min_x:max_x]
            neighbor_min = min_height[min_y:max_y, min_x:max_x]
            neighbor_values = neighbor_min[neighborhood_support]

            if neighbor_values.size < min_neighbor_cells:
                unknown[y_idx, x_idx] = True
                continue

            reference_height = float(np.median(neighbor_values))
            roughness = float(
                np.percentile(neighbor_values, 90) - np.percentile(neighbor_values, 10)
            )
            local_span = float(max_height[y_idx, x_idx] - min_height[y_idx, x_idx])

            if roughness > roughness_threshold or local_span > 0.18:
                unknown[y_idx, x_idx] = True
                continue

            if min_height[y_idx, x_idx] <= reference_height - drop_threshold:
                hazards[y_idx, x_idx] = True

    return hazards, unknown


def filter_clusters(hazards: np.ndarray, min_cluster_cells: int) -> np.ndarray:
    """Drop tiny hazard clusters that look like noise."""
    if not hazards.any():
        return hazards

    filtered = np.zeros_like(hazards)
    visited = np.zeros_like(hazards, dtype=bool)
    offsets = ((1, 0), (-1, 0), (0, 1), (0, -1))

    for start_y, start_x in np.argwhere(hazards):
        if visited[start_y, start_x]:
            continue

        queue: deque[tuple[int, int]] = deque([(start_y, start_x)])
        cluster: list[tuple[int, int]] = []
        visited[start_y, start_x] = True

        while queue:
            y_idx, x_idx = queue.popleft()
            cluster.append((y_idx, x_idx))
            for dy, dx in offsets:
                next_y = y_idx + dy
                next_x = x_idx + dx
                if (
                    next_y < 0
                    or next_y >= hazards.shape[0]
                    or next_x < 0
                    or next_x >= hazards.shape[1]
                    or visited[next_y, next_x]
                    or not hazards[next_y, next_x]
                ):
                    continue
                visited[next_y, next_x] = True
                queue.append((next_y, next_x))

        if len(cluster) >= min_cluster_cells:
            for y_idx, x_idx in cluster:
                filtered[y_idx, x_idx] = True

    return filtered


def pad_hazards(hazards: np.ndarray, padding_cells: int) -> np.ndarray:
    """Dilate hazards slightly to keep the rover off the rim."""
    if padding_cells <= 0 or not hazards.any():
        return hazards

    padded = hazards.copy()
    for y_idx, x_idx in np.argwhere(hazards):
        min_y = max(0, y_idx - padding_cells)
        max_y = min(hazards.shape[0], y_idx + padding_cells + 1)
        min_x = max(0, x_idx - padding_cells)
        max_x = min(hazards.shape[1], x_idx + padding_cells + 1)
        padded[min_y:max_y, min_x:max_x] = True

    return padded


def update_confidence(
    confidence: np.ndarray,
    *,
    evidence: np.ndarray,
    supported: np.ndarray,
    confidence_decay: float,
    confidence_add: float,
    confidence_clear: float,
    publish_threshold: float,
    scale: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Fuse hazard evidence over time to suppress one-frame blips."""
    next_confidence = confidence.copy()
    next_confidence *= confidence_decay**scale
    next_confidence[evidence] += confidence_add * scale
    next_confidence[supported & ~evidence] -= confidence_clear * scale
    next_confidence = np.clip(next_confidence, 0.0, 2.0)
    return next_confidence, next_confidence >= publish_threshold


def decay_confidence(
    confidence: np.ndarray,
    *,
    confidence_decay: float,
    publish_threshold: float,
    scale: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Decay historical hazards when no fresh terrain evidence is available."""
    next_confidence = confidence.copy()
    next_confidence *= confidence_decay**scale
    next_confidence = np.clip(next_confidence, 0.0, 2.0)
    return next_confidence, next_confidence >= publish_threshold


def occupancy_values(
    *,
    hazards: np.ndarray,
    unknown: np.ndarray,
) -> np.ndarray:
    """Render an OccupancyGrid-style tri-state map."""
    grid = np.zeros(hazards.shape, dtype=np.int8)
    grid[unknown] = -1
    grid[hazards] = 100
    return grid
