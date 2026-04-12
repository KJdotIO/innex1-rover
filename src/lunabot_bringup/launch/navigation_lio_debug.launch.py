"""Launch sim, navigation, and a rosbag recorder for manual LIO testing."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_file(package_name: str, *parts: str) -> str:
    """Return a package launch file path."""
    package_root = Path(get_package_share_directory(package_name))
    return str(package_root.joinpath(*parts))


def generate_launch_description():
    bag_output = LaunchConfiguration("bag_output")
    enable_bag_recording = LaunchConfiguration("enable_bag_recording")
    launch_rviz = LaunchConfiguration("launch_rviz")
    sim_settle_delay_s = LaunchConfiguration("sim_settle_delay_s")

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file("lunabot_simulation", "launch", "moon_yard.launch.py")
        )
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file("lunabot_bringup", "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "launch_rviz": launch_rviz,
            "local_odometry_backend": "lio",
        }.items(),
    )

    rosbag_record = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--use-sim-time",
            "-o",
            bag_output,
            "/clock",
            "/tf",
            "/tf_static",
            "/cmd_vel",
            "/cmd_vel_nav",
            "/goal_pose",
            "/plan",
            "/odom",
            "/imu/data_raw",
            "/ouster/points",
            "/odometry/local",
            "/odometry/global",
            "/cloud_registered",
            "/cloud_registered_body",
            "/global_costmap/costmap",
            "/local_costmap/costmap",
            "/rosout",
        ],
        output="screen",
        condition=IfCondition(enable_bag_recording),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_output",
                default_value="bags/navigation_lio_debug",
                description="Output directory for rosbag2 debug recordings.",
            ),
            DeclareLaunchArgument(
                "enable_bag_recording",
                default_value="true",
                description="Record the main localisation and Nav2 topics.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch RViz with the navigation stack.",
            ),
            DeclareLaunchArgument(
                "sim_settle_delay_s",
                default_value="8.0",
                description=(
                    "Seconds to wait after Gazebo starts before launching the "
                    "navigation stack."
                ),
            ),
            simulation_launch,
            TimerAction(period=sim_settle_delay_s, actions=[navigation_launch]),
            TimerAction(period=sim_settle_delay_s, actions=[rosbag_record]),
        ]
    )
