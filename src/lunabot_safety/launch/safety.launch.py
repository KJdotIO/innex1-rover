"""Launch safety input and inhibit bridge nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate the safety launch description."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    estop_backend = LaunchConfiguration("estop_backend")
    estop_gpio_pin = LaunchConfiguration("estop_gpio_pin")
    estop_active_high = LaunchConfiguration("estop_active_high")
    simulated_estop_active = LaunchConfiguration("simulated_estop_active")
    release_debounce_samples = LaunchConfiguration("release_debounce_samples")
    power_inhibit_enabled = LaunchConfiguration("power_inhibit_enabled")

    physical_estop = Node(
        package="lunabot_safety",
        executable="physical_estop_input",
        name="physical_estop_input",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "backend": estop_backend,
                "gpio_pin": ParameterValue(estop_gpio_pin, value_type=int),
                "active_high": ParameterValue(
                    estop_active_high, value_type=bool
                ),
                "simulated_estop_active": ParameterValue(
                    simulated_estop_active, value_type=bool
                ),
                "release_debounce_samples": ParameterValue(
                    release_debounce_samples, value_type=int
                ),
            }
        ],
    )

    estop_node = Node(
        package="lunabot_safety",
        executable="estop_node",
        name="estop_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "power_inhibit_enabled": ParameterValue(
                    power_inhibit_enabled, value_type=bool
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "estop_backend",
                default_value="parameter",
                description="E-stop source: parameter or jetson_gpio.",
            ),
            DeclareLaunchArgument(
                "estop_gpio_pin",
                default_value="0",
                description="Jetson GPIO pin used when estop_backend is jetson_gpio.",
            ),
            DeclareLaunchArgument(
                "estop_active_high",
                default_value="true",
                description="Whether a high GPIO input means E-stop active.",
            ),
            DeclareLaunchArgument(
                "simulated_estop_active",
                default_value="false",
                description="Published E-stop state for parameter backend.",
            ),
            DeclareLaunchArgument(
                "release_debounce_samples",
                default_value="3",
                description="Consecutive clear samples required before release.",
            ),
            DeclareLaunchArgument(
                "power_inhibit_enabled",
                default_value="true",
                description="Latch motion inhibit on critical /power/telemetry.",
            ),
            physical_estop,
            estop_node,
        ]
    )
