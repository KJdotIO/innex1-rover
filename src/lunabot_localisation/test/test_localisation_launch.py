"""Unit tests for localisation launch helper behaviour."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node


def _load_launch_module():
    """Import the localisation launch file as a Python module."""
    module_path = (
        Path(__file__).resolve().parents[1] / "launch" / "localisation.launch.py"
    )
    spec = spec_from_file_location("localisation_launch", module_path)
    assert spec is not None
    assert spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _condition_text(condition) -> str:
    """Return launch condition substitutions as a readable string."""
    return _substitution_text(
        condition.__dict__.get("_IfCondition__predicate_expression", [])
    )


def _substitution_text(value) -> str:
    """Return nested launch substitution internals as readable text."""
    if isinstance(value, list | tuple):
        return "".join(_substitution_text(part) for part in value)
    if "_TextSubstitution__text" in value.__dict__:
        return value.__dict__["_TextSubstitution__text"]
    if "_LaunchConfiguration__variable_name" in value.__dict__:
        return _substitution_text(value.__dict__["_LaunchConfiguration__variable_name"])
    if "_PythonExpression__expression" in value.__dict__:
        return _substitution_text(value.__dict__["_PythonExpression__expression"])
    return str(value)


def _parameter_dict(parameters) -> dict:
    """Return launch node parameters as readable key/value pairs."""
    result = {}
    for group in parameters:
        if not isinstance(group, dict):
            continue
        for key, value in group.items():
            if isinstance(value, list | tuple) or hasattr(value, "__dict__"):
                result[_substitution_text(key)] = _substitution_text(value).splitlines()[
                    0
                ]
            else:
                result[_substitution_text(key)] = value
    return result


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("true", "True"),
        ("True", "True"),
        ("1", "True"),
        ("yes", "True"),
        ("on", "True"),
        ("false", "False"),
        ("False", "False"),
        ("0", "False"),
        ("no", "False"),
        ("off", "False"),
    ],
)
def test_is_truthy_accepts_common_launch_values(text, expected):
    launch_module = _load_launch_module()
    context = LaunchContext()

    expression = launch_module._is_truthy(text)

    assert expression.perform(context) == expected


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("false", "True"),
        ("False", "True"),
        ("0", "True"),
        ("no", "True"),
        ("off", "True"),
        ("true", "False"),
    ],
)
def test_is_falsey_accepts_common_launch_values(text, expected):
    launch_module = _load_launch_module()
    context = LaunchContext()

    expression = launch_module._is_falsey(text)

    assert expression.perform(context) == expected


@pytest.mark.parametrize(
    "text",
    ["maybe", "2", "", "treu"],
)
def test_normalise_bool_text_rejects_invalid_values(text):
    launch_module = _load_launch_module()

    with pytest.raises(ValueError):
        launch_module._normalise_bool_text(text)


def test_tag_pose_bridge_config_switches_between_sim_and_hardware_yaml():
    launch_module = _load_launch_module()
    context = LaunchContext()

    sim_result = launch_module._tag_pose_bridge_config(
        "True",
        "/tmp/tag_pose_bridge_sim.yaml",
        "/tmp/tag_pose_bridge.yaml",
    )
    hardware_result = launch_module._tag_pose_bridge_config(
        "0",
        "/tmp/tag_pose_bridge_sim.yaml",
        "/tmp/tag_pose_bridge.yaml",
    )

    assert sim_result.perform(context) == "/tmp/tag_pose_bridge_sim.yaml"
    assert hardware_result.perform(context) == "/tmp/tag_pose_bridge.yaml"


def test_validate_boolean_launch_arguments_rejects_invalid_launch_value():
    launch_module = _load_launch_module()
    context = LaunchContext()
    context.launch_configurations["lidar_costmap_phase"] = "treu"
    context.launch_configurations["enable_visual_slam"] = "false"
    context.launch_configurations["use_sim_time"] = "true"
    context.launch_configurations["enable_apriltag_debug"] = "false"
    context.launch_configurations["sync_sim_camera_info"] = "false"
    context.launch_configurations["lidar_odometry_backend"] = "none"

    with pytest.raises(ValueError, match="lidar_costmap_phase"):
        launch_module._validate_boolean_launch_arguments(context)


def test_validate_lidar_odometry_backend_rejects_unknown_backend():
    launch_module = _load_launch_module()
    context = LaunchContext()
    context.launch_configurations["lidar_costmap_phase"] = "false"
    context.launch_configurations["enable_visual_slam"] = "false"
    context.launch_configurations["use_sim_time"] = "true"
    context.launch_configurations["enable_apriltag_debug"] = "false"
    context.launch_configurations["sync_sim_camera_info"] = "false"
    context.launch_configurations["lidar_odometry_backend"] = "magic"

    with pytest.raises(ValueError, match="lidar_odometry_backend"):
        launch_module._validate_boolean_launch_arguments(context)


def test_generate_launch_description_has_validation_and_sim_camera_info_aligner(
    monkeypatch: pytest.MonkeyPatch,
):
    launch_module = _load_launch_module()
    monkeypatch.setattr(
        launch_module,
        "get_package_share_directory",
        lambda _package: "/tmp/lunabot_localisation",
    )
    description = launch_module.generate_launch_description()

    validators = [
        entity for entity in description.entities if isinstance(entity, OpaqueFunction)
    ]
    tag_pose_publishers = [
        entity
        for entity in description.entities
        if isinstance(entity, Node)
        and entity.__dict__.get("_Node__node_name") == "tag_pose_publisher"
    ]
    camera_info_aligners = [
        entity
        for entity in description.entities
        if isinstance(entity, Node)
        and entity.__dict__.get("_Node__node_name") == "camera_info_stamp_aligner"
    ]
    legal_lidar_filters = [
        entity
        for entity in description.entities
        if isinstance(entity, Node)
        and entity.__dict__.get("_Node__node_name") == "legal_lidar_filter_ouster"
    ]

    assert len(validators) == 1
    assert len(tag_pose_publishers) == 1
    assert len(camera_info_aligners) == 1
    assert len(legal_lidar_filters) == 1


def test_kiss_icp_uses_legal_lidar_topic_and_disables_tf(
    monkeypatch: pytest.MonkeyPatch,
):
    launch_module = _load_launch_module()
    monkeypatch.setattr(
        launch_module,
        "get_package_share_directory",
        lambda _package: "/tmp/lunabot_localisation",
    )
    description = launch_module.generate_launch_description()

    kiss_nodes = [
        entity
        for entity in description.entities
        if isinstance(entity, Node)
        and entity.__dict__.get("_Node__package") == "kiss_icp"
    ]

    assert len(kiss_nodes) == 1
    remappings = kiss_nodes[0].__dict__.get("_Node__remappings", [])
    parameters = _parameter_dict(kiss_nodes[0].__dict__.get("_Node__parameters", []))
    remapping_text = [
        (_substitution_text(source), _substitution_text(target))
        for source, target in remappings
    ]
    assert ("pointcloud_topic", "legal_lidar_output_topic") in remapping_text
    assert ("kiss/odometry", "/localisation/lidar/odometry") in remapping_text
    assert parameters["lidar_odom_frame"] == "odom"
    assert parameters["publish_odom_tf"] is False


def test_rko_lio_launch_uses_live_os1_imu_topic(
    monkeypatch: pytest.MonkeyPatch,
):
    launch_module = _load_launch_module()
    monkeypatch.setattr(
        launch_module,
        "get_package_share_directory",
        lambda _package: "/tmp/lunabot_localisation",
    )
    description = launch_module.generate_launch_description()

    rko_includes = [
        entity
        for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
        and "odom_topic"
        in dict(entity.__dict__.get("_IncludeLaunchDescription__launch_arguments", []))
    ]

    assert len(rko_includes) == 1
    launch_arguments = dict(
        rko_includes[0].__dict__.get("_IncludeLaunchDescription__launch_arguments", [])
    )
    assert launch_arguments["imu_topic"] == "/ouster/imu"


def test_no_global_ekf_in_launch_description(
    monkeypatch: pytest.MonkeyPatch,
):
    """Verify the global EKF node is not present in the launch description."""
    launch_module = _load_launch_module()
    monkeypatch.setattr(
        launch_module,
        "get_package_share_directory",
        lambda _package: "/tmp/lunabot_localisation",
    )
    description = launch_module.generate_launch_description()

    ekf_map_nodes = [
        entity
        for entity in description.entities
        if isinstance(entity, Node)
        and entity.__dict__.get("_Node__node_name") == "ekf_filter_node_map"
    ]

    assert len(ekf_map_nodes) == 0


def test_rtabmap_node_present_in_launch_description(
    monkeypatch: pytest.MonkeyPatch,
):
    """Verify RTAB-Map SLAM node is included in the launch description."""
    launch_module = _load_launch_module()
    monkeypatch.setattr(
        launch_module,
        "get_package_share_directory",
        lambda _package: "/tmp/lunabot_localisation",
    )
    description = launch_module.generate_launch_description()

    rtabmap_nodes = [
        entity
        for entity in description.entities
        if isinstance(entity, Node)
        and entity.__dict__.get("_Node__node_name") == "rtabmap"
    ]

    assert len(rtabmap_nodes) == 1
    assert "enable_visual_slam" in _condition_text(rtabmap_nodes[0].condition)


def test_identity_map_to_odom_runs_when_visual_slam_is_disabled(
    monkeypatch: pytest.MonkeyPatch,
):
    """The default competition path still has a map->odom TF without RTAB-Map."""
    launch_module = _load_launch_module()
    monkeypatch.setattr(
        launch_module,
        "get_package_share_directory",
        lambda _package: "/tmp/lunabot_localisation",
    )
    description = launch_module.generate_launch_description()

    static_tf_nodes = [
        entity
        for entity in description.entities
        if isinstance(entity, Node)
        and entity.__dict__.get("_Node__package") == "tf2_ros"
        and "--child-frame-id" in entity.__dict__.get("_Node__arguments", [])
        and "odom" in entity.__dict__.get("_Node__arguments", [])
    ]

    assert len(static_tf_nodes) == 1
    assert "enable_visual_slam" in _condition_text(static_tf_nodes[0].condition)


@pytest.mark.parametrize("config_name", ["ekf.yaml", "ekf_lidar_phase.yaml"])
def test_ekf_config_does_not_contain_global_ekf_section(config_name):
    ekf_path = Path(__file__).resolve().parents[1] / "config" / config_name
    config = yaml.safe_load(ekf_path.read_text())

    assert "ekf_filter_node_map" not in config
    assert "ekf_filter_node_odom" in config


def test_rtabmap_config_publishes_map_to_odom_tf():
    rtabmap_path = Path(__file__).resolve().parents[1] / "config" / "rtabmap.yaml"
    config = yaml.safe_load(rtabmap_path.read_text())
    params = config["rtabmap"]["ros__parameters"]

    assert params["publish_tf"] is True
    assert params["publish_tf_odom"] is False
    assert params["subscribe_depth"] is True
    assert params["subscribe_stereo"] is False


def test_rko_lio_config_matches_live_os1_topics_and_frames():
    rko_path = Path(__file__).resolve().parents[1] / "config" / "rko_lio_lunabot.yaml"
    config = yaml.safe_load(rko_path.read_text())

    assert config["lidar_topic"] == "/localisation/lidar/points_legal"
    assert config["imu_topic"] == "/ouster/imu"
    assert config["lidar_frame"] == "os_lidar"
    assert config["imu_frame"] == "os_imu"
    assert config["base_frame"] == "base_footprint"
    assert "extrinsic_lidar2base_quat_xyzw_xyz" in config
    assert "extrinsic_imu2base_quat_xyzw_xyz" in config
