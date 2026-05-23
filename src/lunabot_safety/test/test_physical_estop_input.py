"""Unit tests for physical E-stop input helpers."""

from lunabot_safety.physical_estop_input import (
    PhysicalEstopInput,
    interpret_estop_input,
)


def test_active_high_input_reports_true_when_raw_high():
    assert interpret_estop_input(True, active_high=True) is True


def test_active_high_input_reports_false_when_raw_low():
    assert interpret_estop_input(False, active_high=True) is False


def test_active_low_input_reports_true_when_raw_low():
    assert interpret_estop_input(False, active_high=False) is True


def test_active_low_input_reports_false_when_raw_high():
    assert interpret_estop_input(True, active_high=False) is False


def _node(debounce_samples=3):
    node = object.__new__(PhysicalEstopInput)
    node._published_estop_active = None
    node._release_sample_count = 0
    node._release_debounce_samples = debounce_samples
    return node


def test_estop_assertion_is_immediate():
    node = _node()

    assert node._debounced_estop_active(True) is True


def test_estop_release_is_debounced():
    node = _node(debounce_samples=3)

    assert node._debounced_estop_active(True) is True
    assert node._debounced_estop_active(False) is True
    assert node._debounced_estop_active(False) is True
    assert node._debounced_estop_active(False) is False


def test_estop_reassertion_resets_release_debounce():
    node = _node(debounce_samples=2)

    assert node._debounced_estop_active(True) is True
    assert node._debounced_estop_active(False) is True
    assert node._debounced_estop_active(True) is True
    assert node._debounced_estop_active(False) is True
    assert node._debounced_estop_active(False) is False
