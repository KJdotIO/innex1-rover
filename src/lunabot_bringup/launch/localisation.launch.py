"""Launch file for the localisation stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_file(package_name: str, *parts: str) -> str:
    """Return a package launch path as a string."""
    package_root = Path(get_package_share_directory(package_name))
    return str(package_root.joinpath(*parts))


def generate_launch_description():
    """
    Generate a launch description for the localisation stack.

    Includes the master localisation launch from lunabot_localisation.
    """
    lidar_costmap_phase = LaunchConfiguration("lidar_costmap_phase")
    enable_visual_slam = LaunchConfiguration("enable_visual_slam")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_apriltag_debug = LaunchConfiguration("enable_apriltag_debug")
    local_odometry_backend = LaunchConfiguration("local_odometry_backend")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    localisation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file("lunabot_localisation", "launch", "localisation.launch.py")
        ),
        launch_arguments={
            "lidar_costmap_phase": lidar_costmap_phase,
            "enable_visual_slam": enable_visual_slam,
            "use_sim_time": use_sim_time,
            "enable_apriltag_debug": enable_apriltag_debug,
            "local_odometry_backend": local_odometry_backend,
            "cmd_vel_topic": cmd_vel_topic,
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_costmap_phase",
                default_value="false",
                description="Use odom-only debug localisation with an identity map->odom TF.",
            ),
            DeclareLaunchArgument(
                "enable_visual_slam",
                default_value="false",
                description=(
                    "Optionally enable RTAB-Map odometry and mapping for "
                    "experimentation; the June EKF baseline does not fuse VO."
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock instead of wall time for all launched nodes.",
            ),
            DeclareLaunchArgument(
                "enable_apriltag_debug",
                default_value="false",
                description=(
                    "Launch the apriltag_draw overlay for annotated front camera "
                    "debugging."
                ),
            ),
            DeclareLaunchArgument(
                "local_odometry_backend",
                default_value="ekf",
                description=(
                    "Continuous local odometry backend. Use 'ekf' for the "
                    "wheel-odom baseline or 'lio' for Point-LIO."
                ),
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="cmd_vel",
                description="Velocity command topic used by the start-zone localiser.",
            ),
            localisation_launch,
        ]
    )
