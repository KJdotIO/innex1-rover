"""Unit checks for the OAK-D Pro launch wrapper."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


def _load_launch_module():
    """Import the launch file as a Python module for unit testing."""
    module_path = (
        Path(__file__).resolve().parents[1] / "launch" / "oakd_pro.launch.py"
    )
    spec = importlib.util.spec_from_file_location("oakd_pro_launch", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


oak_launch = _load_launch_module()


def test_profile_settings_accept_supported_profiles():
    """Supported profiles should return the expected launch settings."""
    assert oak_launch._profile_settings("usb2_degraded") == {
        "params_file": "oakd_usb2_degraded.yaml",
        "pointcloud_enable": "false",
    }
    assert oak_launch._profile_settings("usb3_full") == {
        "params_file": "oakd_usb3_full.yaml",
        "pointcloud_enable": "true",
    }


def test_profile_settings_reject_unknown_profile():
    """Unknown profiles should be rejected before launch starts."""
    with pytest.raises(ValueError, match="Unsupported OAK-D Pro profile"):
        oak_launch._profile_settings("definitely_wrong")


def test_require_non_empty_rejects_blank_values():
    """Blank topic or frame arguments should fail fast."""
    with pytest.raises(ValueError, match="must not be empty"):
        oak_launch._require_non_empty("rgb_image_topic", " ")


def test_existing_package_file_checks_resolved_path(monkeypatch, tmp_path: Path):
    """Package file resolution should fail clearly when a file is missing."""
    config_dir = tmp_path / "config"
    config_dir.mkdir()
    config_file = config_dir / "oakd_usb2_degraded.yaml"
    config_file.write_text("dummy: true\n", encoding="utf-8")

    monkeypatch.setattr(
        oak_launch,
        "get_package_share_directory",
        lambda _package_name: str(tmp_path),
    )

    resolved = oak_launch._existing_package_file(
        "lunabot_sensors", "config/oakd_usb2_degraded.yaml"
    )
    assert resolved == config_file

    with pytest.raises(FileNotFoundError, match="Required file"):
        oak_launch._existing_package_file("lunabot_sensors", "launch/missing.launch.py")
