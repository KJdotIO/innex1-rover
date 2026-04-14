"""Launch joystick teleoperation nodes for Lunabot."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _config_path(*parts: str) -> str:
    """Return a teleop package path as a string for launch parameters."""
    return str(Path(get_package_share_directory("lunabot_teleop")).joinpath(*parts))


def _validate_non_negative_int(value: str, argument_name: str) -> str:
    """Return a normalised integer string or raise on invalid input."""
    try:
        parsed = int(str(value).strip())
    except ValueError as error:
        raise ValueError(
            f"Expected an integer launch value for '{argument_name}', got '{value}'."
        ) from error
    if parsed < 0:
        raise ValueError(
            f"Expected '{argument_name}' to be zero or greater, got '{value}'."
        )
    return str(parsed)


def _validate_launch_arguments(context):
    """Reject invalid launch arguments before any teleop nodes start."""
    joy_device_name = LaunchConfiguration("joy_device_name").perform(context).strip()
    if not joy_device_name:
        _validate_non_negative_int(
            LaunchConfiguration("joy_device_id").perform(context),
            argument_name="joy_device_id",
        )
    return []


def _create_game_controller_node(context, use_sim_time):
    """Create joy node with name-based selection fallback."""
    joy_device_name = LaunchConfiguration("joy_device_name").perform(context).strip()
    base_params = [
        {"use_sim_time": use_sim_time},
        {"deadzone": 0.08},
        {"autorepeat_rate": 20.0},
        {"coalesce_interval_ms": 1},
    ]
    if joy_device_name:
        base_params.append({"device_name": joy_device_name})
    else:
        joy_device_id = _validate_non_negative_int(
            LaunchConfiguration("joy_device_id").perform(context),
            argument_name="joy_device_id",
        )
        base_params.append({"device_id": int(joy_device_id)})
    return [
        Node(
            package="joy",
            executable="game_controller_node",
            name="game_controller_node",
            output="screen",
            parameters=base_params,
        )
    ]


def generate_launch_description():
    """
    Generate a launch description for joystick teleoperation.

    Start the SDL-backed game controller node and teleop_twist_joy with the
    Lunabot Xbox-style mapping. Teleop commands are published on
    /cmd_vel_teleop so a mux can arbitrate between manual and autonomous input.
    """
    teleop_config = _config_path("config", "xbox_teleop.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    teleop_twist = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        output="screen",
        parameters=[teleop_config, {"use_sim_time": use_sim_time}],
        remappings=[("cmd_vel", "cmd_vel_teleop")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description=(
                    "Use /clock instead of wall time for teleop nodes."
                ),
            ),
            DeclareLaunchArgument(
                "joy_device_id",
                default_value="0",
                description=("SDL device index for the connected controller."),
            ),
            DeclareLaunchArgument(
                "joy_device_name",
                default_value="",
                description=(
                    "Optional SDL controller name to match (takes precedence over "
                    "joy_device_id when set)."
                ),
            ),
            OpaqueFunction(function=_validate_launch_arguments),
            OpaqueFunction(
                function=lambda context: _create_game_controller_node(
                    context, use_sim_time
                )
            ),
            teleop_twist,
        ]
    )
