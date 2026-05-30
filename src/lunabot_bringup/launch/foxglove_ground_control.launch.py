"""Launch Foxglove ground control with compressed camera topics only."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from lunabot_bringup.runtime_profile import default_profiles_path, load_profiles


def _profile_allowlist(context) -> list[str]:
    profile_name = LaunchConfiguration("profile").perform(context)
    profiles = load_profiles(default_profiles_path())
    if profile_name not in profiles:
        available = ", ".join(sorted(profiles))
        raise ValueError(
            f"Unknown Foxglove runtime profile '{profile_name}'. "
            f"Available profiles: {available}."
        )
    return list(profiles[profile_name].foxglove_allowlist)


def _compressed_republisher(
    *,
    name: str,
    input_topic: str,
    output_topic: str,
    transport: str,
    use_sim_time: LaunchConfiguration,
    condition: IfCondition,
    jpeg_quality: LaunchConfiguration | None = None,
) -> Node:
    parameters = [{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}]
    if jpeg_quality is not None:
        parameters[0]["out.format"] = "jpeg"
        parameters[0]["out.jpeg_quality"] = ParameterValue(
            jpeg_quality,
            value_type=int,
        )

    return Node(
        package="image_transport",
        executable="republish",
        name=name,
        output="screen",
        arguments=[
            "raw",
            transport,
            "--ros-args",
            "--remap",
            f"in:={input_topic}",
            "--remap",
            f"out/{transport}:={output_topic}",
        ],
        parameters=parameters,
        condition=condition,
    )


def _launch_nodes(context):
    allowlist = _profile_allowlist(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    jpeg_quality = LaunchConfiguration("jpeg_quality")
    port = LaunchConfiguration("port")
    send_buffer_limit = LaunchConfiguration("send_buffer_limit")

    front_camera_republisher = _compressed_republisher(
        name="camera_front_compressed_republisher",
        input_topic="/camera_front/image",
        output_topic="/camera_front/image/compressed",
        transport="compressed",
        use_sim_time=use_sim_time,
        jpeg_quality=jpeg_quality,
        condition=IfCondition(LaunchConfiguration("enable_front_camera")),
    )

    rear_camera_republisher = _compressed_republisher(
        name="camera_rear_compressed_republisher",
        input_topic="/camera_rear/image",
        output_topic="/camera_rear/image/compressed",
        transport="compressed",
        use_sim_time=use_sim_time,
        jpeg_quality=jpeg_quality,
        condition=IfCondition(LaunchConfiguration("enable_rear_camera")),
    )

    front_depth_republisher = _compressed_republisher(
        name="camera_front_depth_compressed_republisher",
        input_topic="/camera_front/depth_image",
        output_topic="/camera_front/depth_image/compressedDepth",
        transport="compressedDepth",
        use_sim_time=use_sim_time,
        condition=IfCondition(LaunchConfiguration("enable_front_depth")),
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "port": ParameterValue(port, value_type=int),
                "send_buffer_limit": ParameterValue(
                    send_buffer_limit,
                    value_type=int,
                ),
                "topic_whitelist": allowlist,
                "capabilities": [],
            }
        ],
    )

    return [
        front_camera_republisher,
        rear_camera_republisher,
        front_depth_republisher,
        foxglove_bridge,
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate a Foxglove bridge launch description for operator telemetry."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="sim_competition",
                description="Runtime profile whose Foxglove allowlist should be exposed.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock for the camera republishers and bridge.",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="8765",
                description="Foxglove WebSocket port.",
            ),
            DeclareLaunchArgument(
                "send_buffer_limit",
                default_value="10000000",
                description=(
                    "Maximum Foxglove per-client send buffer in bytes. "
                    "Keep payloads lean instead of relying on this."
                ),
            ),
            DeclareLaunchArgument(
                "jpeg_quality",
                default_value="50",
                description="JPEG quality for compressed camera republishing.",
            ),
            DeclareLaunchArgument(
                "enable_front_camera",
                default_value="true",
                description="Republish /camera_front/image as compressed JPEG.",
            ),
            DeclareLaunchArgument(
                "enable_rear_camera",
                default_value="false",
                description="Republish /camera_rear/image as compressed JPEG.",
            ),
            DeclareLaunchArgument(
                "enable_front_depth",
                default_value="false",
                description="Republish /camera_front/depth_image as compressedDepth.",
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
