from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

TRUTHY_VALUES = ("1", "true", "yes", "on")
FALSEY_VALUES = ("0", "false", "no", "off")


def _config_path(*parts: str) -> str:
    """Return a localisation package path as a string for launch parameters."""
    package_root = Path(get_package_share_directory("lunabot_localisation"))
    return str(package_root.joinpath(*parts))


def _normalise_bool_text(value, argument_name=None):
    """Return a normalised launch boolean string or raise on invalid input."""
    normalised = str(value).strip().lower()
    if normalised in TRUTHY_VALUES or normalised in FALSEY_VALUES:
        return normalised
    if argument_name is not None:
        raise ValueError(
            f"Expected a boolean-style launch value for '{argument_name}', "
            f"got '{value}'."
        )
    raise ValueError(f"Expected a boolean-style launch value, got '{value}'.")


def _is_truthy(value):
    """Return a launch expression that accepts common truthy strings."""
    return PythonExpression(
        [
            "'",
            value,
            f"'.strip().lower() in {list(TRUTHY_VALUES)}",
        ]
    )


def _is_falsey(value):
    """Return a launch expression that accepts common falsey strings."""
    return PythonExpression(
        [
            "'",
            value,
            f"'.strip().lower() in {list(FALSEY_VALUES)}",
        ]
    )


def _tag_pose_bridge_config(use_sim_time, sim_config, hardware_config):
    """Return the correct tag-pose bridge config for the selected clock mode."""
    return PythonExpression(
        [
            "'",
            sim_config,
            "' if ",
            _is_truthy(use_sim_time),
            " else '",
            hardware_config,
            "'",
        ]
    )


def _validate_boolean_launch_arguments(context):
    """Reject invalid boolean-style launch argument values early."""
    for argument_name in (
        "lidar_costmap_phase",
        "use_sim_time",
        "enable_apriltag_debug",
    ):
        _normalise_bool_text(
            LaunchConfiguration(argument_name).perform(context),
            argument_name=argument_name,
        )
    return []


def generate_launch_description():
    """
    Generate a launch description for the localisation stack.

    Local EKF provides odom -> base_footprint from wheel odometry + IMU.
    RTAB-Map SLAM provides map -> odom correction via RGB-D loop closure
    and AprilTag landmarks. The global EKF has been removed; RTAB-Map
    handles drift correction directly.
    """
    rtabmap_yaml = _config_path("config", "rtabmap.yaml")
    ekf_lidar_phase_yaml = _config_path("config", "ekf_lidar_phase.yaml")
    apriltag_yaml = _config_path("config", "apriltag.yaml")
    tag_pose_bridge_yaml = _config_path("config", "tag_pose_bridge.yaml")
    tag_pose_bridge_sim_yaml = _config_path("config", "tag_pose_bridge_sim.yaml")
    start_zone_localisation_yaml = _config_path(
        "config",
        "start_zone_localisation.yaml",
    )
    lidar_costmap_phase = LaunchConfiguration("lidar_costmap_phase")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_apriltag_debug = LaunchConfiguration("enable_apriltag_debug")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    tag_map_x = LaunchConfiguration("tag_map_x")
    tag_map_y = LaunchConfiguration("tag_map_y")
    tag_map_z = LaunchConfiguration("tag_map_z")
    tag_map_yaw = LaunchConfiguration("tag_map_yaw")

    normal_mode = IfCondition(_is_falsey(lidar_costmap_phase))
    apriltag_debug_condition = IfCondition(
        PythonExpression(
            [
                _is_falsey(lidar_costmap_phase),
                " and ",
                _is_truthy(enable_apriltag_debug),
            ]
        )
    )

    camera_remappings = [
        ("rgb/image", "/camera_front/image"),
        ("depth/image", "/camera_front/depth_image"),
        ("rgb/camera_info", "/camera_front/camera_info"),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_costmap_phase",
                default_value="false",
                description=(
                    "Use odom-only debug localisation with an identity map->odom TF."
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description=("Use /clock instead of wall time for all launched nodes."),
            ),
            DeclareLaunchArgument(
                "enable_apriltag_debug",
                default_value="false",
                description=(
                    "Launch the apriltag_draw overlay for annotated front "
                    "camera debugging."
                ),
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="cmd_vel",
                description=(
                    "Velocity command topic used by the start-zone localiser."
                ),
            ),
            DeclareLaunchArgument(
                "tag_map_x",
                default_value="0.0",
                description=("Configured map-frame x position of the start-zone tag."),
            ),
            DeclareLaunchArgument(
                "tag_map_y",
                default_value="1.08",
                description=("Configured map-frame y position of the start-zone tag."),
            ),
            DeclareLaunchArgument(
                "tag_map_z",
                default_value="0.25",
                description=("Configured map-frame z position of the start-zone tag."),
            ),
            DeclareLaunchArgument(
                "tag_map_yaw",
                default_value="0.0",
                description=("Configured map-frame yaw of the start-zone tag."),
            ),
            OpaqueFunction(function=_validate_boolean_launch_arguments),
            # --- Local EKF: odom -> base_footprint ---
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[
                    ekf_lidar_phase_yaml,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("odometry/filtered", "/odometry/local"),
                    ("set_pose", "/ekf_filter_node_odom/set_pose"),
                ],
            ),
            # --- Debug: identity map -> odom for costmap-only testing ---
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "odom",
                ],
                condition=IfCondition(_is_truthy(lidar_costmap_phase)),
            ),
            # --- RTAB-Map SLAM: map -> odom via loop closure ---
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                parameters=[
                    rtabmap_yaml,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    *camera_remappings,
                    ("odom", "/odometry/local"),
                    ("tag_detections", "/camera_front/tags"),
                ],
                arguments=["-d"],
                condition=normal_mode,
            ),
            # --- AprilTag detector ---
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                name="apriltag",
                output="screen",
                parameters=[
                    apriltag_yaml,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("image_rect", "/camera_front/image"),
                    ("camera_info", "/camera_front/camera_info"),
                    ("detections", "/camera_front/tags"),
                ],
                condition=normal_mode,
            ),
            # --- AprilTag debug overlay (optional) ---
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("apriltag_draw"),
                            "launch",
                            "draw.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "camera": "/camera_front",
                    "image": "image",
                    "tags": "tags",
                    "image_transport": "raw",
                }.items(),
                condition=apriltag_debug_condition,
            ),
            # --- Static TF: map -> tag (known arena placement) ---
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    tag_map_x,
                    "--y",
                    tag_map_y,
                    "--z",
                    tag_map_z,
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    tag_map_yaw,
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "tag36h11:0",
                ],
                condition=normal_mode,
            ),
            # --- Tag pose bridge: publishes /tag_pose for diagnostics ---
            Node(
                package="lunabot_localisation",
                executable="tag_pose_publisher",
                name="tag_pose_publisher",
                output="screen",
                parameters=[
                    _tag_pose_bridge_config(
                        use_sim_time,
                        tag_pose_bridge_sim_yaml,
                        tag_pose_bridge_yaml,
                    ),
                    {
                        "use_sim_time": use_sim_time,
                        "detections_topic": "/camera_front/tags",
                    },
                ],
                condition=normal_mode,
            ),
            # --- Start-zone localiser: tag search + readiness gate ---
            Node(
                package="lunabot_localisation",
                executable="start_zone_localiser",
                name="start_zone_localiser",
                output="screen",
                parameters=[
                    start_zone_localisation_yaml,
                    {
                        "use_sim_time": use_sim_time,
                        "cmd_vel_topic": cmd_vel_topic,
                    },
                ],
                condition=normal_mode,
            ),
        ]
    )
