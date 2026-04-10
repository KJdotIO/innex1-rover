"""Launch file for the mission manager."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate a launch description for the mission manager."""
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Start only the standalone mission manager; dependent action servers stay separate.
    mission_manager = Node(
        package="lunabot_bringup",
        executable="mission_manager",
        name="mission_manager",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock instead of wall time for the mission manager.",
            ),
            mission_manager,
        ]
    )
