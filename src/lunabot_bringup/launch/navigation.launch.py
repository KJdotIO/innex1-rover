"""Launch file for the navigation stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import SetRemap

from lunabot_bringup.launch_gate import select_preflight_config_path


def _is_truthy(value):
    """Return a launch expression that accepts common truthy strings."""
    return PythonExpression(
        [
            "'",
            value,
            "'.strip().lower() in ['1', 'true', 'yes', 'on']",
        ]
    )


def _is_falsey(value):
    """Return a launch expression that accepts common falsey strings."""
    return PythonExpression(
        [
            "'",
            value,
            "'.strip().lower() in ['0', 'false', 'no', 'off']",
        ]
    )


def _build_nav2_start_actions(
    pkg_bringup, nav_params_path, use_sim_time, enable_teleop
):
    """Return Nav2 start actions for direct or muxed operation."""
    nav2_launch = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, "launch", "nav2_navigation.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": nav_params_path,
                    "autostart": "true",
                }.items(),
            ),
        ],
        condition=UnlessCondition(enable_teleop),
    )

    nav2_launch_group = GroupAction(
        [
            SetRemap(src="cmd_vel", dst="cmd_vel_nav"),
            SetRemap(src="cmd_vel_smoothed", dst="cmd_vel_nav"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, "launch", "nav2_navigation.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": nav_params_path,
                    "autostart": "true",
                }.items(),
            ),
        ],
        condition=IfCondition(enable_teleop),
    )

    return [nav2_launch, nav2_launch_group]


def _select_localiser_cmd_vel_topic(enable_teleop):
    """Return the command topic used by the start-zone localiser."""
    return PythonExpression(
        [
            "'cmd_vel_nav' if '",
            enable_teleop,
            "'.strip().lower() in ['1', 'true', 'yes', 'on'] else 'cmd_vel'",
        ]
    )


def _handle_preflight_exit(
    event,
    _context,
    pkg_bringup,
    nav_params_path,
    use_sim_time,
    enable_teleop,
):
    """Start Nav2 only when the launch-gate preflight passes."""
    if event.returncode == 0:
        return _build_nav2_start_actions(
            pkg_bringup, nav_params_path, use_sim_time, enable_teleop
        )

    return [
        LogInfo(
            msg=(
                "Preflight launch gate failed; Nav2 will not start. "
                "Fix the reported readiness checks and relaunch."
            )
        ),
        OpaqueFunction(function=_raise_preflight_launch_failure),
    ]


def _raise_preflight_launch_failure(_context):
    """Abort launch with a non-zero exit when the preflight gate fails."""
    raise RuntimeError("Navigation preflight launch gate failed")


def generate_launch_description():
    """
    Generate a launch description for the navigation stack.

    Start the blank map server, localisation include, and Nav2 navigation
    servers. The forwarded launch arguments keep the odom-only debug mode
    available while AprilTag global localisation remains the default path.
    Optionally launch RViz and joystick teleop, then arbitrate between
    autonomous and manual velocity commands through a twist mux.
    """
    # Locate the configuration files
    pkg_bringup = get_package_share_directory("lunabot_bringup")
    pkg_nav = get_package_share_directory("lunabot_navigation")
    pkg_teleop = get_package_share_directory("lunabot_teleop")

    nav_params_path = os.path.join(pkg_nav, "config", "nav2_params.yaml")
    blank_map_path = os.path.join(pkg_nav, "maps", "moon_yard_blank.yaml")
    rviz_config_path = os.path.join(pkg_bringup, "rviz", "navigation.rviz")
    twist_mux_params_path = os.path.join(
        pkg_bringup, "config", "twist_mux.yaml"
    )
    preflight_config_path = os.path.join(
        pkg_bringup, "config", "preflight_checks.yaml"
    )
    preflight_lidar_debug_config_path = os.path.join(
        pkg_bringup, "config", "preflight_checks_lidar_debug.yaml"
    )
    default_preflight_config = select_preflight_config_path(
        False,
        preflight_config_path,
        preflight_lidar_debug_config_path,
    )
    default_preflight_lidar_debug_config = select_preflight_config_path(
        True,
        preflight_config_path,
        preflight_lidar_debug_config_path,
    )
    lidar_costmap_phase = LaunchConfiguration("lidar_costmap_phase")
    enable_visual_slam = LaunchConfiguration("enable_visual_slam")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_apriltag_debug = LaunchConfiguration("enable_apriltag_debug")
    enable_teleop = LaunchConfiguration("enable_teleop")
    enforce_preflight = LaunchConfiguration("enforce_preflight")
    preflight_config = LaunchConfiguration("preflight_config")
    preflight_lidar_debug_config = LaunchConfiguration(
        "preflight_lidar_debug_config"
    )
    joy_device_id = LaunchConfiguration("joy_device_id")
    localiser_cmd_vel_topic = _select_localiser_cmd_vel_topic(enable_teleop)

    localisation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, "launch", "localisation.launch.py")
        ),
        launch_arguments={
            "lidar_costmap_phase": lidar_costmap_phase,
            "enable_visual_slam": enable_visual_slam,
            "use_sim_time": use_sim_time,
            "enable_apriltag_debug": enable_apriltag_debug,
            "cmd_vel_topic": localiser_cmd_vel_topic,
        }.items(),
    )

    navigate_to_pose_gate = Node(
        package="lunabot_bringup",
        executable="navigate_to_pose_gate",
        name="navigate_to_pose_gate",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {
                "gate_enabled": PythonExpression(
                    [
                        "(",
                        _is_falsey(lidar_costmap_phase),
                        ")",
                    ]
                ),
                "readiness_timeout_s": 5.0,
            },
        ],
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_teleop, "launch", "joystick_teleop.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "joy_device_id": joy_device_id,
        }.items(),
        condition=IfCondition(enable_teleop),
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"yaml_filename": blank_map_path},
        ],
    )

    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(launch_rviz),
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        remappings=[("cmd_vel_out", "cmd_vel")],
        parameters=[twist_mux_params_path, {"use_sim_time": use_sim_time}],
        condition=IfCondition(enable_teleop),
    )

    preflight_gate = Node(
        package="lunabot_bringup",
        executable="preflight_check",
        name="preflight_gate",
        output="screen",
        arguments=[
            "--phase",
            "launch",
            "--use-sim-time",
            use_sim_time,
            "--config",
            preflight_config,
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "(",
                    _is_truthy(enforce_preflight),
                    ") and (",
                    _is_falsey(lidar_costmap_phase),
                    ")",
                ]
            )
        ),
    )

    preflight_gate_lidar_debug = Node(
        package="lunabot_bringup",
        executable="preflight_check",
        name="preflight_gate_lidar_debug",
        output="screen",
        arguments=[
            "--phase",
            "launch",
            "--use-sim-time",
            use_sim_time,
            "--config",
            preflight_lidar_debug_config,
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "(",
                    _is_truthy(enforce_preflight),
                    ") and (",
                    _is_truthy(lidar_costmap_phase),
                    ")",
                ]
            )
        ),
    )

    preflight_gate_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=preflight_gate,
            on_exit=lambda event, context: _handle_preflight_exit(
                event,
                context,
                pkg_bringup,
                nav_params_path,
                use_sim_time,
                enable_teleop,
            ),
        ),
        condition=IfCondition(_is_truthy(enforce_preflight)),
    )

    preflight_gate_lidar_debug_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=preflight_gate_lidar_debug,
            on_exit=lambda event, context: _handle_preflight_exit(
                event,
                context,
                pkg_bringup,
                nav_params_path,
                use_sim_time,
                enable_teleop,
            ),
        ),
        condition=IfCondition(_is_truthy(enforce_preflight)),
    )

    direct_nav2_start = GroupAction(
        _build_nav2_start_actions(
            pkg_bringup, nav_params_path, use_sim_time, enable_teleop
        ),
        condition=IfCondition(_is_falsey(enforce_preflight)),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_costmap_phase",
                default_value="false",
                description=(
                    "Use odom-only debug localisation with an identity "
                    "map->odom TF."
                ),
            ),
            DeclareLaunchArgument(
                "enable_visual_slam",
                default_value="false",
                description=(
                    "Optionally enable RTAB-Map visual odometry "
                    "alongside AprilTag global localisation."
                ),
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="false",
                description=(
                    "Launch RViz with sim time enabled using the repo's "
                    "navigation config."
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description=(
                    "Use /clock instead of wall time for all launched nodes."
                ),
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
                "enable_teleop",
                default_value="false",
                description=(
                    "Launch joystick teleoperation through twist_mux."
                ),
            ),
            DeclareLaunchArgument(
                "enforce_preflight",
                default_value="true",
                description=(
                    "Run the launch-gate preflight checks before starting Nav2."
                ),
            ),
            DeclareLaunchArgument(
                "preflight_config",
                default_value=default_preflight_config,
                description=(
                    "Preflight config used for the normal launch-gate path."
                ),
            ),
            DeclareLaunchArgument(
                "preflight_lidar_debug_config",
                default_value=default_preflight_lidar_debug_config,
                description=(
                    "Preflight config used when lidar_costmap_phase is enabled."
                ),
            ),
            DeclareLaunchArgument(
                "joy_device_id",
                default_value="0",
                description=(
                    "SDL device index for the connected controller."
                ),
            ),
            map_server,
            map_lifecycle_manager,
            localisation_launch,
            navigate_to_pose_gate,
            teleop_launch,
            twist_mux,
            preflight_gate,
            preflight_gate_lidar_debug,
            preflight_gate_handler,
            preflight_gate_lidar_debug_handler,
            direct_nav2_start,
            rviz,
        ]
    )
