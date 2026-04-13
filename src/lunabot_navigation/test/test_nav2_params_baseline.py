"""Regression tests for the June Nav2 behaviour tree baseline."""

from pathlib import Path

import yaml


def test_june_baseline_uses_stock_nav2_bt():
    nav2_params_path = (
        Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"
    )
    config = yaml.safe_load(nav2_params_path.read_text())
    bt_xml = config["bt_navigator"]["ros__parameters"]["default_bt_xml_filename"]

    assert "nav2_bt_navigator" in bt_xml
    assert "navigate_to_pose_w_replanning_and_recovery" in bt_xml
