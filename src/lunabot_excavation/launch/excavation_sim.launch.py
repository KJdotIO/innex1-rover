"""Launch the excavation sim proxy with the controller and action server."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate a launch description for the excavation sim stack."""
    force_overcurrent = LaunchConfiguration("force_overcurrent")
    force_driver_fault = LaunchConfiguration("force_driver_fault")
    hold_home_switch_false = LaunchConfiguration("hold_home_switch_false")

    return LaunchDescription(
        [
            DeclareLaunchArgument("force_overcurrent", default_value="false"),
            DeclareLaunchArgument("force_driver_fault", default_value="false"),
            DeclareLaunchArgument("hold_home_switch_false", default_value="false"),
            Node(
                package="lunabot_excavation",
                executable="excavation_sim_proxy",
                name="excavation_sim_proxy",
                output="screen",
                parameters=[
                    {
                        "force_overcurrent": ParameterValue(
                            force_overcurrent,
                            value_type=bool,
                        ),
                        "force_driver_fault": ParameterValue(
                            force_driver_fault,
                            value_type=bool,
                        ),
                        "hold_home_switch_false": ParameterValue(
                            hold_home_switch_false,
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="lunabot_excavation",
                executable="excavation_controller",
                name="excavation_controller",
                output="screen",
            ),
            Node(
                package="lunabot_excavation",
                executable="excavation_action_server",
                name="excavation_action_server",
                output="screen",
            ),
        ]
    )
