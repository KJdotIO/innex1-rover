"""Launch the full simulation stack and run one mission dry run."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Compose the simulation stack and the one-shot mission harness."""
    pkg_bringup = get_package_share_directory("lunabot_bringup")
    pkg_control = get_package_share_directory("lunabot_control")
    pkg_excavation = get_package_share_directory("lunabot_excavation")
    pkg_simulation = get_package_share_directory("lunabot_simulation")

    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    force_overcurrent = LaunchConfiguration("force_overcurrent")
    force_driver_fault = LaunchConfiguration("force_driver_fault")
    hold_home_switch_false = LaunchConfiguration("hold_home_switch_false")
    travel_frame_id = LaunchConfiguration("travel_frame_id")
    travel_x_m = LaunchConfiguration("travel_x_m")
    travel_y_m = LaunchConfiguration("travel_y_m")
    travel_yaw_rad = LaunchConfiguration("travel_yaw_rad")

    moon_yard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation, "launch", "moon_yard.launch.py")
        )
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "launch_rviz": launch_rviz,
            "enforce_preflight": "true",
        }.items(),
    )

    excavation_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_excavation, "launch", "excavation_sim.launch.py")
        ),
        launch_arguments={
            "force_overcurrent": force_overcurrent,
            "force_driver_fault": force_driver_fault,
            "hold_home_switch_false": hold_home_switch_false,
        }.items(),
    )

    deposit_action_server = Node(
        package="lunabot_control",
        executable="material_action_server",
        name="material_action_server",
        output="screen",
    )

    mission_dry_run = Node(
        package="lunabot_bringup",
        executable="mission_dry_run",
        name="mission_dry_run",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "travel_frame_id": travel_frame_id,
                "travel_x_m": travel_x_m,
                "travel_y_m": travel_y_m,
                "travel_yaw_rad": travel_yaw_rad,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("force_overcurrent", default_value="false"),
            DeclareLaunchArgument("force_driver_fault", default_value="false"),
            DeclareLaunchArgument("hold_home_switch_false", default_value="false"),
            DeclareLaunchArgument("travel_frame_id", default_value="map"),
            DeclareLaunchArgument("travel_x_m", default_value="0.0"),
            DeclareLaunchArgument("travel_y_m", default_value="0.0"),
            DeclareLaunchArgument("travel_yaw_rad", default_value="0.0"),
            moon_yard,
            navigation,
            excavation_sim,
            deposit_action_server,
            mission_dry_run,
        ]
    )
