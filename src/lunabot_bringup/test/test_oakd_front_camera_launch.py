"""Tests for the OAK-D front camera launch file."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def _load_launch_module():
    """Import the OAK-D camera launch file as a Python module."""
    module_path = (
        Path(__file__).resolve().parents[1] / "launch" / "oakd_front_camera.launch.py"
    )
    spec = spec_from_file_location("oakd_front_camera_launch", module_path)
    assert spec is not None
    assert spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_generate_launch_description_exposes_hardware_camera_arguments(
    monkeypatch: pytest.MonkeyPatch,
):
    launch_module = _load_launch_module()
    monkeypatch.setattr(
        launch_module,
        "get_package_share_directory",
        lambda package: f"/tmp/{package}",
    )

    description = launch_module.generate_launch_description()
    argument_names = [
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    validators = [
        entity for entity in description.entities if isinstance(entity, OpaqueFunction)
    ]

    assert argument_names == [
        "driver_name",
        "camera_model",
        "parent_frame",
        "params_file",
        "enable_depth",
        "enable_pointcloud",
        "use_rectified_rgb",
    ]
    assert len(validators) == 1


def test_launch_setup_rejects_invalid_depth_toggle(monkeypatch: pytest.MonkeyPatch):
    launch_module = _load_launch_module()
    monkeypatch.setattr(
        launch_module,
        "get_package_share_directory",
        lambda package: f"/tmp/{package}",
    )
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "driver_name": "oak_front",
            "camera_model": "OAK-D-PRO",
            "parent_frame": "camera_front_link",
            "params_file": "/tmp/oakd_front.yaml",
            "enable_depth": "sometimes",
            "enable_pointcloud": "false",
            "use_rectified_rgb": "true",
        }
    )

    with pytest.raises(ValueError, match="enable_depth"):
        launch_module._launch_setup(context)


def test_launch_setup_returns_grouped_depthai_include(monkeypatch: pytest.MonkeyPatch):
    launch_module = _load_launch_module()
    monkeypatch.setattr(
        launch_module,
        "get_package_share_directory",
        lambda package: f"/tmp/{package}",
    )
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "driver_name": "oak_front",
            "camera_model": "OAK-D-PRO",
            "parent_frame": "camera_front_link",
            "params_file": "/tmp/oakd_front.yaml",
            "enable_depth": "true",
            "enable_pointcloud": "true",
            "use_rectified_rgb": "true",
        }
    )

    actions = launch_module._launch_setup(context)

    assert len(actions) == 1
