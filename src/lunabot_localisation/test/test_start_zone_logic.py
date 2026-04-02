"""Behaviour tests for the start-zone stable-lock tracker."""

from lunabot_localisation.start_zone_logic import PoseSample
from lunabot_localisation.start_zone_logic import StableLockTracker


def _sample(stamp_ns: int, x: float, y: float, yaw: float) -> PoseSample:
    return PoseSample(stamp_ns=stamp_ns, x=x, y=y, yaw=yaw)


def test_tracker_accepts_stable_lock_window():
    tracker = StableLockTracker()
    for idx in range(5):
        tracker.add_sample(_sample(idx * 250_000_000, 1.0, 2.0, 0.1))

    assert tracker.has_stable_lock(
        now_ns=1_000_000_000,
        window_ns=1_000_000_000,
        min_samples=5,
        max_gap_ns=250_000_000,
        max_translation_spread_m=0.15,
        max_yaw_spread_rad=0.2,
    )


def test_tracker_accepts_window_with_timer_slack():
    tracker = StableLockTracker()
    for idx in range(5):
        tracker.add_sample(_sample(idx * 250_000_000, 1.0, 2.0, 0.1))

    assert tracker.has_stable_lock(
        now_ns=1_010_000_000,
        window_ns=1_000_000_000,
        min_samples=4,
        max_gap_ns=250_000_000,
        max_translation_spread_m=0.15,
        max_yaw_spread_rad=0.2,
    )


def test_tracker_rejects_short_burst_inside_window():
    tracker = StableLockTracker()
    for stamp_ns in (700_000_000, 800_000_000, 900_000_000, 1_000_000_000):
        tracker.add_sample(_sample(stamp_ns, 1.0, 2.0, 0.1))

    assert not tracker.has_stable_lock(
        now_ns=1_010_000_000,
        window_ns=800_000_000,
        min_samples=4,
        max_gap_ns=350_000_000,
        max_translation_spread_m=0.15,
        max_yaw_spread_rad=0.2,
    )


def test_tracker_rejects_large_sample_gap():
    tracker = StableLockTracker()
    tracker.add_sample(_sample(0, 1.0, 2.0, 0.1))
    tracker.add_sample(_sample(250_000_000, 1.0, 2.0, 0.1))
    tracker.add_sample(_sample(900_000_000, 1.0, 2.0, 0.1))
    tracker.add_sample(_sample(1_150_000_000, 1.0, 2.0, 0.1))
    tracker.add_sample(_sample(1_400_000_000, 1.0, 2.0, 0.1))

    assert not tracker.has_stable_lock(
        now_ns=1_400_000_000,
        window_ns=1_000_000_000,
        min_samples=5,
        max_gap_ns=250_000_000,
        max_translation_spread_m=0.15,
        max_yaw_spread_rad=0.2,
    )


def test_tracker_rejects_translation_spread():
    tracker = StableLockTracker()
    for idx in range(5):
        tracker.add_sample(_sample(idx * 250_000_000, 1.0 + (0.05 * idx), 2.0, 0.1))

    assert not tracker.has_stable_lock(
        now_ns=1_000_000_000,
        window_ns=1_000_000_000,
        min_samples=5,
        max_gap_ns=250_000_000,
        max_translation_spread_m=0.15,
        max_yaw_spread_rad=0.2,
    )


def test_tracker_rejects_yaw_spread():
    tracker = StableLockTracker()
    for idx in range(5):
        tracker.add_sample(_sample(idx * 250_000_000, 1.0, 2.0, 0.1 + (0.06 * idx)))

    assert not tracker.has_stable_lock(
        now_ns=1_000_000_000,
        window_ns=1_000_000_000,
        min_samples=5,
        max_gap_ns=250_000_000,
        max_translation_spread_m=0.15,
        max_yaw_spread_rad=0.2,
    )


def test_tracker_rejects_future_sample_as_fresh():
    tracker = StableLockTracker()
    tracker.add_sample(_sample(2_000_000_000, 1.0, 2.0, 0.1))

    assert not tracker.is_fresh(now_ns=1_000_000_000, max_gap_ns=250_000_000)


def test_tracker_prunes_future_samples():
    tracker = StableLockTracker()
    tracker.add_sample(_sample(500_000_000, 1.0, 2.0, 0.1))
    tracker.add_sample(_sample(2_000_000_000, 1.0, 2.0, 0.1))

    tracker.prune_future(1_000_000_000)

    latest = tracker.latest()
    assert latest is not None
    assert latest.stamp_ns == 500_000_000
