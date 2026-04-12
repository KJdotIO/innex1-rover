"""Pure helpers for start-zone localisation stability checks."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class PoseSample:
    """One accepted map-frame pose sample from the tag bridge."""

    stamp_ns: int
    x: float
    y: float
    yaw: float


def angle_delta(a: float, b: float) -> float:
    """Return the wrapped angular difference between two yaw values."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class StableLockTracker:
    """Track recent accepted tag poses and decide when the lock is stable."""

    def __init__(self) -> None:
        self._samples: deque[PoseSample] = deque()

    def clear(self) -> None:
        """Discard all tracked samples."""
        self._samples.clear()

    def add_sample(self, sample: PoseSample) -> None:
        """Append a newly accepted pose sample."""
        self._samples.append(sample)

    def prune(self, min_stamp_ns: int) -> None:
        """Discard samples older than the requested lower time bound."""
        while self._samples and self._samples[0].stamp_ns < min_stamp_ns:
            self._samples.popleft()

    def prune_future(self, max_stamp_ns: int) -> None:
        """Discard samples that lie beyond the current time horizon."""
        while self._samples and self._samples[-1].stamp_ns > max_stamp_ns:
            self._samples.pop()

    def latest(self) -> PoseSample | None:
        """Return the newest available sample, if any."""
        if not self._samples:
            return None
        return self._samples[-1]

    def is_fresh(self, now_ns: int, max_gap_ns: int) -> bool:
        """Return whether the most recent sample is recent enough."""
        latest = self.latest()
        if latest is None:
            return False
        if latest.stamp_ns > now_ns:
            return False
        return (now_ns - latest.stamp_ns) <= max_gap_ns

    def has_stable_lock(
        self,
        *,
        now_ns: int,
        window_ns: int,
        min_samples: int,
        max_gap_ns: int,
        max_translation_spread_m: float,
        max_yaw_spread_rad: float,
    ) -> bool:
        """Return whether the recent accepted samples satisfy the lock rule."""
        self.prune(now_ns - window_ns)
        if len(self._samples) < min_samples:
            return False

        oldest = self._samples[0]
        newest = self._samples[-1]
        if newest.stamp_ns > now_ns:
            return False
        required_span_ns = max(0, window_ns - max_gap_ns)
        if newest.stamp_ns - oldest.stamp_ns < required_span_ns:
            return False

        prev_stamp_ns = None
        for sample in self._samples:
            if prev_stamp_ns is not None and sample.stamp_ns - prev_stamp_ns > max_gap_ns:
                return False
            prev_stamp_ns = sample.stamp_ns

        if self.translation_spread() > max_translation_spread_m:
            return False

        return not self.yaw_spread() > max_yaw_spread_rad

    def translation_spread(self) -> float:
        """Return the maximum planar distance between tracked samples."""
        max_distance = 0.0
        samples = list(self._samples)
        for index, sample in enumerate(samples):
            for other in samples[index + 1:]:
                dx = other.x - sample.x
                dy = other.y - sample.y
                max_distance = max(max_distance, math.hypot(dx, dy))
        return max_distance

    def yaw_spread(self) -> float:
        """Return the maximum wrapped yaw difference across tracked samples."""
        max_delta = 0.0
        samples = list(self._samples)
        for index, sample in enumerate(samples):
            for other in samples[index + 1:]:
                max_delta = max(max_delta, abs(angle_delta(other.yaw, sample.yaw)))
        return max_delta
