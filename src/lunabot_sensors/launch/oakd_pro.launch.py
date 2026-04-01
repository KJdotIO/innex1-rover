"""Launch the OAK-D Pro on the repo's stable front-camera contract."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _validate_profile(context, *_args, **_kwargs):
    """Fail early when the requested OAK profile is unsupported."""
    profile = LaunchConfiguration("profile").perform(context)
    if profile in {"usb2_degraded", "usb3_full"}:
        return []
    raise RuntimeError(
        "Unsupported OAK-D Pro profile '"
        f"{profile}'. Expected 'usb2_degraded' or 'usb3_full'."
    )


def generate_launch_description():
    """Generate the hardware OAK-D Pro launch description."""
    pkg_sensors = FindPackageShare("lunabot_sensors")

    profile = LaunchConfiguration("profile")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rgb_image_topic = LaunchConfiguration("rgb_image_topic")
    rgb_camera_info_topic = LaunchConfiguration("rgb_camera_info_topic")
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    optical_frame_id = LaunchConfiguration("optical_frame_id")
    point_cloud_frame_id = LaunchConfiguration("point_cloud_frame_id")

    usb2_params = PathJoinSubstitution(
        [pkg_sensors, "config", "oakd_usb2_degraded.yaml"]
    )
    usb3_params = PathJoinSubstitution(
        [pkg_sensors, "config", "oakd_usb3_full.yaml"]
    )
    depthai_launch = PathJoinSubstitution(
        [FindPackageShare("depthai_ros_driver_v3"), "launch", "driver.launch.py"]
    )

    usb2_condition = IfCondition(
        PythonExpression(["'", profile, "' == 'usb2_degraded'"])
    )
    usb3_condition = IfCondition(PythonExpression(["'", profile, "' == 'usb3_full'"]))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="usb2_degraded",
                description="DepthAI hardware profile to launch.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use wall time for real hardware by default.",
            ),
            DeclareLaunchArgument(
                "rgb_image_topic",
                default_value="/oak/rgb/image_raw",
                description="Upstream RGB image topic from depthai_ros_driver_v3.",
            ),
            DeclareLaunchArgument(
                "rgb_camera_info_topic",
                default_value="/oak/rgb/camera_info",
                description="Upstream RGB camera info topic from depthai_ros_driver_v3.",
            ),
            DeclareLaunchArgument(
                "depth_image_topic",
                default_value="/oak/stereo/image_raw",
                description="Upstream depth image topic from depthai_ros_driver_v3.",
            ),
            DeclareLaunchArgument(
                "point_cloud_topic",
                default_value="/oak/points",
                description="Upstream point cloud topic from depthai_ros_driver_v3.",
            ),
            DeclareLaunchArgument(
                "optical_frame_id",
                default_value="camera_front_optical_frame",
                description="Public optical frame exposed to the rest of the stack.",
            ),
            DeclareLaunchArgument(
                "point_cloud_frame_id",
                default_value="",
                description=(
                    "Optional override for the public point cloud frame. "
                    "Leave empty unless the cloud transform has been verified."
                ),
            ),
            OpaqueFunction(function=_validate_profile),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(depthai_launch),
                launch_arguments={
                    "params_file": usb2_params,
                    "pointcloud.enable": "false",
                }.items(),
                condition=usb2_condition,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(depthai_launch),
                launch_arguments={
                    "params_file": usb3_params,
                    "pointcloud.enable": "true",
                }.items(),
                condition=usb3_condition,
            ),
            Node(
                package="lunabot_sensors",
                executable="camera_contract_adapter",
                name="camera_contract_adapter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "rgb_image_topic": rgb_image_topic,
                        "rgb_camera_info_topic": rgb_camera_info_topic,
                        "depth_image_topic": depth_image_topic,
                        "point_cloud_topic": point_cloud_topic,
                        "optical_frame_id": optical_frame_id,
                        "point_cloud_frame_id": point_cloud_frame_id,
                    }
                ],
            ),
        ]
    )
