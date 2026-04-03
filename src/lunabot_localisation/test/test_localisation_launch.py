"""Unit tests for localisation launch helper behaviour."""

from importlib.util import module_from_spec
from importlib.util import spec_from_file_location
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def _load_launch_module():
    """Import the localisation launch file as a Python module."""
    module_path = (
        Path(__file__).resolve().parents[1] / "launch" / "localisation.launch.py"
    )
    spec = spec_from_file_location("localisation_launch", module_path)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _ekf_parameters(config_name, node_name):
    """Return EKF ros__parameters from a package config file."""
    config_path = Path(__file__).resolve().parents[1] / "config" / config_name
    config = yaml.safe_load(config_path.read_text())
    return config[node_name]["ros__parameters"]


def _tf_owner_link(parameters):
    """Return the TF link owned by EKF publish_tf/world_frame settings."""
    assert parameters["publish_tf"] is True

    world_frame = parameters["world_frame"]
    odom_frame = parameters["odom_frame"]
    base_link_frame = parameters["base_link_frame"]

    if world_frame == odom_frame:
        return (odom_frame, base_link_frame)

    return (world_frame, odom_frame)


def _node_name(node):
    """Return a launch node's configured name."""
    return node.__dict__.get("_Node__node_name")


def _node_package(node):
    """Return a launch node's package name."""
    return node.node_package


def _node_executable(node):
    """Return a launch node's executable name."""
    return node.node_executable


def _node_parameter_file_names(node):
    """Return configured parameter-file basenames for a launch node."""
    names = []
    for parameter_source in node.__dict__.get("_Node__parameters", ()):
        param_file = getattr(parameter_source, "param_file", None)
        if param_file is None:
            continue
        names.append(Path(str(param_file)).name)
    return tuple(names)


def _static_tf_owner_link(node):
    """Return a static transform link when the node publishes one explicitly."""
    arguments = node.__dict__.get("_Node__arguments", ())
    argument_map = dict(zip(arguments[::2], arguments[1::2]))
    parent = argument_map.get("--frame-id")
    child = argument_map.get("--child-frame-id")
    if parent is None or child is None:
        return None
    return (parent, child)


def _configured_tf_owner_link(node):
    """Return the TF link published by a launch node when it is explicit."""
    if (
        _node_package(node) == "tf2_ros"
        and _node_executable(node) == "static_transform_publisher"
    ):
        return _static_tf_owner_link(node)

    if (
        _node_package(node) != "robot_localization"
        or _node_executable(node) != "ekf_node"
    ):
        return None

    node_name = _node_name(node)
    for config_name in _node_parameter_file_names(node):
        try:
            parameters = _ekf_parameters(config_name, node_name)
        except (FileNotFoundError, KeyError):
            continue
        if parameters.get("publish_tf") is True:
            return _tf_owner_link(parameters)
    return None


def _runtime_tf_owner_links(*, lidar_costmap_phase):
    """Return the effective TF owner links for the selected launch mode."""
    local_parameters = _ekf_parameters(
        "ekf_lidar_phase.yaml", "ekf_filter_node_odom"
    )
    owner_links = {_tf_owner_link(local_parameters)}

    if lidar_costmap_phase:
        owner_links.add(("map", "odom"))
    else:
        global_parameters = _ekf_parameters("ekf.yaml", "ekf_filter_node_map")
        owner_links.add(_tf_owner_link(global_parameters))

    return owner_links


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

    with pytest.raises(ValueError, match="lidar_costmap_phase"):
        launch_module._validate_boolean_launch_arguments(context)


def test_generate_launch_description_has_validation_and_one_tag_pose_node():
    launch_module = _load_launch_module()
    launch_module.get_package_share_directory = (
        lambda _package: "/tmp/lunabot_localisation"
    )
    description = launch_module.generate_launch_description()

    validators = [
        entity
        for entity in description.entities
        if isinstance(entity, OpaqueFunction)
    ]
    tag_pose_publishers = [
        entity
        for entity in description.entities
        if isinstance(entity, Node)
        and entity.__dict__.get("_Node__node_name") == "tag_pose_publisher"
    ]

    assert len(validators) == 1
    assert len(tag_pose_publishers) == 1


def test_runtime_dual_ekf_tf_contract_assigns_one_owner_per_link():
    local_parameters = _ekf_parameters(
        "ekf_lidar_phase.yaml", "ekf_filter_node_odom"
    )
    global_parameters = _ekf_parameters("ekf.yaml", "ekf_filter_node_map")

    assert local_parameters["odom_frame"] == "odom"
    assert local_parameters["base_link_frame"] == "base_footprint"
    assert local_parameters["world_frame"] == "odom"

    assert global_parameters["map_frame"] == "map"
    assert global_parameters["odom_frame"] == "odom"
    assert global_parameters["base_link_frame"] == "base_footprint"
    assert global_parameters["world_frame"] == "map"
    assert global_parameters["odom0"] == "/odometry/local"

    owners = {
        _tf_owner_link(local_parameters): "local_ekf",
        _tf_owner_link(global_parameters): "global_ekf",
    }

    assert owners == {
        ("odom", "base_footprint"): "local_ekf",
        ("map", "odom"): "global_ekf",
    }


@pytest.mark.parametrize(
    ("lidar_costmap_phase", "expected_links"),
    [
        (False, {("odom", "base_footprint"), ("map", "odom")}),
        (True, {("odom", "base_footprint"), ("map", "odom")}),
    ],
)
def test_runtime_tf_owner_links_remain_unique_across_modes(
    lidar_costmap_phase, expected_links
):
    owner_links = _runtime_tf_owner_links(
        lidar_costmap_phase=lidar_costmap_phase
    )

    assert owner_links == expected_links
    assert len(owner_links) == len(expected_links)


@pytest.mark.parametrize(
    ("lidar_costmap_phase", "expected_links"),
    [
        ("true", {("odom", "base_footprint"), ("map", "odom")}),
        ("1", {("odom", "base_footprint"), ("map", "odom")}),
        ("false", {("odom", "base_footprint"), ("map", "odom")}),
        ("0", {("odom", "base_footprint"), ("map", "odom")}),
    ],
)
def test_launch_exposes_only_expected_tf_owner_links(
    lidar_costmap_phase, expected_links
):
    launch_module = _load_launch_module()
    launch_module.get_package_share_directory = (
        lambda _package: "/tmp/lunabot_localisation"
    )
    description = launch_module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations["lidar_costmap_phase"] = lidar_costmap_phase
    context.launch_configurations["enable_visual_slam"] = "false"
    context.launch_configurations["use_sim_time"] = "true"
    context.launch_configurations["enable_apriltag_debug"] = "false"

    owner_links = [
        _configured_tf_owner_link(entity)
        for entity in description.entities
        if isinstance(entity, Node)
        and (
            entity.condition is None
            or entity.condition.evaluate(context)
        )
    ]
    owner_links = [owner_link for owner_link in owner_links if owner_link is not None]

    assert set(owner_links) == expected_links
    assert len(owner_links) == len(expected_links)


@pytest.mark.parametrize(
    ("lidar_costmap_phase", "expected_static", "expected_global_ekf"),
    [
        ("true", True, False),
        ("1", True, False),
        ("false", False, True),
        ("0", False, True),
    ],
)
def test_launch_switches_between_global_ekf_and_debug_map_to_odom(
    lidar_costmap_phase, expected_static, expected_global_ekf
):
    launch_module = _load_launch_module()
    launch_module.get_package_share_directory = (
        lambda _package: "/tmp/lunabot_localisation"
    )
    description = launch_module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations["lidar_costmap_phase"] = lidar_costmap_phase
    context.launch_configurations["enable_visual_slam"] = "false"
    context.launch_configurations["use_sim_time"] = "true"
    context.launch_configurations["enable_apriltag_debug"] = "false"

    static_map_to_odom_publishers = [
        entity
        for entity in description.entities
        if isinstance(entity, Node)
        and _node_package(entity) == "tf2_ros"
        and _node_executable(entity) == "static_transform_publisher"
        and _static_tf_owner_link(entity) == ("map", "odom")
    ]
    global_ekf_nodes = [
        entity
        for entity in description.entities
        if isinstance(entity, Node)
        and _node_package(entity) == "robot_localization"
        and _node_executable(entity) == "ekf_node"
        and _node_name(entity) == "ekf_filter_node_map"
    ]

    assert len(static_map_to_odom_publishers) == 1
    assert len(global_ekf_nodes) == 1

    static_condition = static_map_to_odom_publishers[0].condition
    global_condition = global_ekf_nodes[0].condition

    assert static_condition is not None
    assert global_condition is not None
    assert static_condition.evaluate(context) is expected_static
    assert global_condition.evaluate(context) is expected_global_ekf
    assert static_condition.evaluate(context) is not global_condition.evaluate(context)


@pytest.mark.parametrize("config_name", ["ekf.yaml", "ekf_lidar_phase.yaml"])
def test_june_baseline_ekf_does_not_fuse_visual_odometry(config_name):
    odom_parameters = _ekf_parameters(config_name, "ekf_filter_node_odom")

    assert "odom1" not in odom_parameters
    assert "/visual_odometry" not in odom_parameters.values()
