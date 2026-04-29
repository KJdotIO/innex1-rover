"""Launch the front OAK-D Pro camera into the rover camera topic contract."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap

from lunabot_bringup.oakd_camera_contract import (
    camera_launch_arguments,
    camera_topic_remappings,
    normalise_bool_text,
    select_depthai_launch_file,
)


def _package_path(package_name: str) -> Path:
    """Return a package share directory as a Path."""
    return Path(get_package_share_directory(package_name))


def _config_path(*parts: str) -> str:
    """Return a lunabot_bringup config path as a string."""
    return str(_package_path("lunabot_bringup").joinpath(*parts))


def _validate_bool_launch_argument(context, argument_name: str) -> bool:
    """Validate and return a launch boolean argument."""
    return normalise_bool_text(
        LaunchConfiguration(argument_name).perform(context),
        argument_name,
    )


def _launch_setup(context):
    """Resolve launch arguments and return the DepthAI include group."""
    enable_depth = _validate_bool_launch_argument(context, "enable_depth")
    enable_pointcloud = _validate_bool_launch_argument(context, "enable_pointcloud")
    use_rectified_rgb = _validate_bool_launch_argument(context, "use_rectified_rgb")

    driver_name = LaunchConfiguration("driver_name").perform(context).strip()
    if not driver_name:
        raise ValueError("Expected non-empty launch value for 'driver_name'.")

    camera_model = LaunchConfiguration("camera_model").perform(context).strip()
    if not camera_model:
        raise ValueError("Expected non-empty launch value for 'camera_model'.")

    parent_frame = LaunchConfiguration("parent_frame").perform(context).strip()
    if not parent_frame:
        raise ValueError("Expected non-empty launch value for 'parent_frame'.")

    params_file = LaunchConfiguration("params_file").perform(context).strip()
    if not params_file:
        raise ValueError("Expected non-empty launch value for 'params_file'.")

    depthai_launch = _package_path("depthai_ros_driver").joinpath(
        "launch",
        select_depthai_launch_file(enable_pointcloud),
    )
    remap_actions = [
        SetRemap(src=source, dst=target)
        for source, target in camera_topic_remappings(
            driver_name=driver_name,
            use_rectified_rgb=use_rectified_rgb,
            enable_depth=enable_depth,
            enable_pointcloud=enable_pointcloud,
        )
    ]
    depthai_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(depthai_launch)),
        launch_arguments=camera_launch_arguments(
            driver_name=driver_name,
            camera_model=camera_model,
            parent_frame=parent_frame,
            params_file=params_file,
            enable_depth=enable_depth,
            enable_pointcloud=enable_pointcloud,
            use_rectified_rgb=use_rectified_rgb,
        ).items(),
    )
    return [GroupAction([*remap_actions, depthai_include])]


def generate_launch_description():
    """Generate the OAK-D Pro front camera hardware launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "driver_name",
                default_value="oak_front",
                description="DepthAI camera node name before rover topic remapping.",
            ),
            DeclareLaunchArgument(
                "camera_model",
                default_value="OAK-D-PRO",
                description="DepthAI camera model passed to depthai_ros_driver.",
            ),
            DeclareLaunchArgument(
                "parent_frame",
                default_value="camera_front_link",
                description="Parent frame for DepthAI camera description TF.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=_config_path("config", "oakd_front.yaml"),
                description="DepthAI ROS driver parameter file.",
            ),
            DeclareLaunchArgument(
                "enable_depth",
                default_value="true",
                description="Publish /camera_front/depth_image when true.",
            ),
            DeclareLaunchArgument(
                "enable_pointcloud",
                default_value="false",
                description="Publish /camera_front/points when true.",
            ),
            DeclareLaunchArgument(
                "use_rectified_rgb",
                default_value="true",
                description=(
                    "Remap DepthAI rectified RGB into /camera_front/image. "
                    "Set false to use the raw RGB stream."
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
