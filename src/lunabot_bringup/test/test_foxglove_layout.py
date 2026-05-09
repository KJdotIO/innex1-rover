import json
from pathlib import Path

LAYOUT_PATH = (
    Path(__file__).resolve().parents[3]
    / "docs"
    / "foxglove"
    / "mission_control.layout.json"
)


def _walk_panels(node):
    if isinstance(node, dict):
        if node.get("type") == "panel":
            yield node
        for value in node.values():
            yield from _walk_panels(value)
    elif isinstance(node, list):
        for item in node:
            yield from _walk_panels(item)


def test_mission_control_layout_is_valid_json():
    layout = json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))

    assert layout["name"] == "INNEX-1 Mission Control"
    assert layout["content"]["type"] == "split"


def test_mission_control_layout_covers_operator_topics():
    layout = json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))

    topics = set(layout["topics"])

    assert "/mission/state" in topics
    assert "/safety/motion_inhibit" in topics
    assert "/diagnostics" in topics
    assert "/drivetrain/status" in topics
    assert "/excavation/status" in topics
    assert "/localisation/start_zone_status" in topics
    assert "/power/telemetry" in topics
    assert "/camera_front/image" in topics


def test_mission_control_layout_keeps_debug_streams_out():
    layout = json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))
    topics = set(layout["topics"])

    assert "/ouster/points" not in topics
    assert "/global_costmap/costmap" not in topics
    assert "/local_costmap/costmap" not in topics


def test_mission_control_layout_has_expected_panel_groups():
    layout = json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))
    panels = {panel["title"]: panel for panel in _walk_panels(layout["content"])}

    assert {
        "Front Camera",
        "Diagnostics",
        "Mission",
        "Safety and Power",
        "Subsystems",
    }.issubset(panels)
