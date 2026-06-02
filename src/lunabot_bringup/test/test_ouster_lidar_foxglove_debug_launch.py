import json
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "ouster_lidar_foxglove_debug.launch.py"
CONFIG_PATH = PACKAGE_ROOT / "config" / "ouster_lidar_debug.yaml"
LAYOUT_PATH = REPO_ROOT / "docs" / "foxglove" / "ouster_lidar_debug.layout.json"


def test_ouster_debug_launch_includes_driver_and_foxglove_bridge():
    launch_text = LAUNCH_PATH.read_text(encoding="utf-8")

    assert "driver.launch.py" in launch_text
    assert 'package="foxglove_bridge"' in launch_text
    assert "/ouster/points" in launch_text
    assert "/ouster/imu" in launch_text
    assert "topic_whitelist" in launch_text


def test_ouster_debug_config_matches_successful_bringup_topics():
    config_text = CONFIG_PATH.read_text(encoding="utf-8")

    assert "sensor_hostname: os-122610007923.local" in config_text
    assert "udp_dest: 192.168.8.20" in config_text
    assert "lidar_mode: 1024x10" in config_text
    assert "udp_profile_lidar: RNG15_RFL8_NIR8" in config_text
    assert "proc_mask: IMU|PCL|SCAN|TLM" in config_text
    assert "point_type: native" in config_text
    assert "v_reduction: 4" in config_text


def test_ouster_debug_layout_covers_lidar_topics():
    layout = json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))
    topics = set()
    for config in layout["configById"].values():
        topic = config.get("topicPath") or config.get("cameraTopic")
        if topic:
            topics.add(topic)
        topics.update(config.get("topics", {}))

    assert "/ouster/points" in topics
    assert "/ouster/imu" in topics
    assert "/ouster/telemetry" in topics
