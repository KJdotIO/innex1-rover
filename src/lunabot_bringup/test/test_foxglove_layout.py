import json
from pathlib import Path

LAYOUT_PATH = (
    Path(__file__).resolve().parents[3]
    / "docs"
    / "foxglove"
    / "mission_control.layout.json"
)


def _walk_layout_ids(node):
    if isinstance(node, dict):
        for value in node.values():
            yield from _walk_layout_ids(value)
    elif isinstance(node, list):
        for item in node:
            yield from _walk_layout_ids(item)
    elif isinstance(node, str):
        yield node


def _configured_topics(layout):
    topics = set()
    for config in layout["configById"].values():
        topic = config.get("topicPath") or config.get("cameraTopic")
        if topic:
            topics.add(topic)
    return topics


def test_mission_control_layout_is_valid_json():
    layout = json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))

    assert isinstance(layout["configById"], dict)
    assert isinstance(layout["layout"], dict)


def test_mission_control_layout_covers_operator_topics():
    layout = json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))

    topics = _configured_topics(layout)

    assert "/mission/state" in topics
    assert "/safety/motion_inhibit" in topics
    assert "/diagnostics" in topics
    assert "/drivetrain/status" in topics
    assert "/excavation/status" in topics
    assert "/localisation/start_zone_status" in topics
    assert "/power/telemetry" in topics
    assert "/camera_front/image/compressed" in topics


def test_mission_control_layout_keeps_debug_streams_out():
    layout = json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))
    topics = _configured_topics(layout)

    assert "/camera_front/image" not in topics
    assert "/ouster/points" not in topics
    assert "/global_costmap/costmap" not in topics
    assert "/local_costmap/costmap" not in topics


def test_mission_control_layout_has_expected_panel_groups():
    layout = json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))
    panels = set(_walk_layout_ids(layout["layout"]))

    assert {
        "Image!front_camera",
        "RawMessages!diagnostics",
        "RawMessages!mission_state",
        "RawMessages!motion_inhibit",
        "RawMessages!power",
        "RawMessages!drivetrain",
        "RawMessages!excavation",
    }.issubset(panels)
