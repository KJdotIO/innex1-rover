"""Behaviour tests for joystick teleop launch validation."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest


def _load_launch_module():
    """Import the joystick teleop launch file as a Python module."""
    pytest.importorskip("launch")
    pytest.importorskip("ament_index_python.packages")
    pytest.importorskip("launch_ros.actions")
    pytest.importorskip("launch_ros.parameter_descriptions")
    module_path = (
        Path(__file__).resolve().parents[1] / "launch" / "joystick_teleop.launch.py"
    )
    spec = spec_from_file_location("joystick_teleop_launch", module_path)
    assert spec is not None
    assert spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("0", "0"),
        (" 7 ", "7"),
        ("12", "12"),
    ],
)
def test_validate_non_negative_int_accepts_non_negative_values(text, expected):
    launch_module = _load_launch_module()

    assert (
        launch_module._validate_non_negative_int(text, argument_name="joy_device_id")
        == expected
    )


@pytest.mark.parametrize("text", ["-1", "left", "1.5", ""])
def test_validate_non_negative_int_rejects_invalid_values(text):
    launch_module = _load_launch_module()

    with pytest.raises(ValueError):
        launch_module._validate_non_negative_int(text, argument_name="joy_device_id")


def test_validate_launch_arguments_rejects_invalid_device_id():
    launch_module = _load_launch_module()
    launch_context_module = pytest.importorskip("launch.launch_context")
    context = launch_context_module.LaunchContext()
    context.launch_configurations["joy_device_id"] = "-1"

    with pytest.raises(ValueError, match="joy_device_id"):
        launch_module._validate_launch_arguments(context)


def test_validate_launch_arguments_accepts_invalid_id_when_name_is_set():
    launch_module = _load_launch_module()
    launch_context_module = pytest.importorskip("launch.launch_context")
    context = launch_context_module.LaunchContext()
    context.launch_configurations["joy_device_id"] = "not-an-int"
    context.launch_configurations["joy_device_name"] = "Xbox Series X Controller"

    launch_module._validate_launch_arguments(context)
