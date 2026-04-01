"""Launch the OAK-D Pro on the repo's stable front-camera contract."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
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


SUPPORTED_PROFILES = {
    "usb2_degraded": {
        "params_file": "oakd_usb2_degraded.yaml",
        "pointcloud_enable": "false",
    },
    "usb3_full": {
        "params_file": "oakd_usb3_full.yaml",
        "pointcloud_enable": "true",
    },
}


def _profile_settings(profile: str) -> dict[str, str]:
    """Return launch settings for a supported OAK profile."""
    try:
        return SUPPORTED_PROFILES[profile]
    except KeyError as exc:
        raise ValueError(
            "Unsupported OAK-D Pro profile "
            f"'{profile}'. Expected one of: {', '.join(SUPPORTED_PROFILES)}."
        ) from exc


def _require_non_empty(name: str, value: str) -> None:
    """Reject empty launch arguments that would produce silent miswiring."""
    if value.strip():
        return
    raise ValueError(f"Launch argument '{name}' must not be empty.")


def _existing_package_file(package_name: str, relative_path: str) -> Path:
    """Resolve a package file and raise a clear error if it is missing."""
    package_share = Path(get_package_share_directory(package_name))
    resolved = package_share / relative_path
    if resolved.exists():
        return resolved
    raise FileNotFoundError(
        f"Required file '{relative_path}' was not found in package "
        f"'{package_name}' ({package_share})."
    )


def _validate_launch_environment(context, *_args, **_kwargs):
    """Fail early when launch prerequisites are missing or misconfigured."""
    profile = LaunchConfiguration("profile").perform(context)
    _profile_settings(profile)

    required_args = {
        "rgb_image_topic": LaunchConfiguration("rgb_image_topic").perform(context),
        "rgb_camera_info_topic": LaunchConfiguration("rgb_camera_info_topic").perform(
            context
        ),
        "depth_image_topic": LaunchConfiguration("depth_image_topic").perform(context),
        "point_cloud_topic": LaunchConfiguration("point_cloud_topic").perform(context),
        "optical_frame_id": LaunchConfiguration("optical_frame_id").perform(context),
    }
    for name, value in required_args.items():
        _require_non_empty(name, value)

    _existing_package_file("lunabot_sensors", "config/oakd_usb2_degraded.yaml")
    _existing_package_file("lunabot_sensors", "config/oakd_usb3_full.yaml")
    _existing_package_file("depthai_ros_driver_v3", "launch/driver.launch.py")
    return []


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
    startup_warn_timeout_s = LaunchConfiguration("startup_warn_timeout_s")

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
            DeclareLaunchArgument(
                "startup_warn_timeout_s",
                default_value="10.0",
                description=(
                    "Seconds to wait before warning that the required RGB streams "
                    "have not appeared yet."
                ),
            ),
            OpaqueFunction(function=_validate_launch_environment),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(depthai_launch),
                launch_arguments={
                    "params_file": usb2_params,
                    "pointcloud.enable": _profile_settings("usb2_degraded")[
                        "pointcloud_enable"
                    ],
                    "publish_tf_from_calibration": "false",
                }.items(),
                condition=usb2_condition,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(depthai_launch),
                launch_arguments={
                    "params_file": usb3_params,
                    "pointcloud.enable": _profile_settings("usb3_full")[
                        "pointcloud_enable"
                    ],
                    "publish_tf_from_calibration": "false",
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
                        "startup_warn_timeout_s": startup_warn_timeout_s,
                    }
                ],
            ),
        ]
    )
