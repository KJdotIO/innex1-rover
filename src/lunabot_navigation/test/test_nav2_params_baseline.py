"""Regression tests for the Nav2 behaviour tree baseline."""

from pathlib import Path

import yaml


def test_uses_bounded_recovery_bt():
    nav2_params_path = (
        Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"
    )
    config = yaml.safe_load(nav2_params_path.read_text())
    bt_xml = config["bt_navigator"]["ros__parameters"]["default_bt_xml_filename"]

    assert "navigate_to_pose_bounded_recovery" in bt_xml


def test_bounded_recovery_bt_exists():
    bt_path = (
        Path(__file__).resolve().parents[1]
        / "behavior_trees"
        / "navigate_to_pose_bounded_recovery.xml"
    )
    assert bt_path.exists(), f"Expected BT XML at {bt_path}"

    content = bt_path.read_text()
    assert 'number_of_retries="3"' in content, (
        "Outer RecoveryNode should limit retries to 3"
    )
    assert "ComputePathToPose" in content
    assert "FollowPath" in content
