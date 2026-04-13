import os
import platform
import tempfile
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _package_path(package_name: str) -> Path:
    """Return the package share directory as a Path."""
    return Path(get_package_share_directory(package_name))


def _patched_world_path(world_path: Path) -> Path:
    """Swap Ogre2 for Ogre on macOS and keep the original world elsewhere."""
    if platform.system() != "Darwin":
        return world_path

    content = world_path.read_text()
    if "<render_engine>ogre2</render_engine>" not in content:
        return world_path

    patched_content = content.replace(
        "<render_engine>ogre2</render_engine>",
        "<render_engine>ogre</render_engine>",
    )
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        prefix="moon_yard_mac_",
        suffix=".sdf",
        delete=False,
    ) as temp_file:
        temp_file.write(patched_content)

    patched_world_path = Path(temp_file.name)
    print(f"macOS detected: using patched Ogre1 world at {patched_world_path}")
    return patched_world_path


def _prepend_resource_path(models_path: Path) -> None:
    """Add the simulation models path to GZ_SIM_RESOURCE_PATH once."""
    path_text = str(models_path)
    resource_paths = [
        entry
        for entry in os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(os.pathsep)
        if entry
    ]
    if path_text not in resource_paths:
        os.environ["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
            [path_text, *resource_paths]
        )


def _resolve_world(context):
    """Resolve the world file and return the gz_sim launch action."""
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim").find("ros_gz_sim")
    pkg_lunabot_simulation = _package_path("lunabot_simulation")

    world_name = LaunchConfiguration("world").perform(context).strip()
    world_path = _patched_world_path(
        pkg_lunabot_simulation / "worlds" / f"{world_name}.sdf"
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py")
            ),
            launch_arguments={"gz_args": f"-r -s '{world_path}'"}.items(),
        )
    ]


def generate_launch_description():
    pkg_lunabot_description = _package_path("lunabot_description")
    pkg_lunabot_simulation = _package_path("lunabot_simulation")

    _prepend_resource_path(pkg_lunabot_simulation / "models")

    robot_description = xacro.process(
        str(pkg_lunabot_description / "urdf" / "lunabot.urdf.xacro")
    )

    # Spawn position - surface mesh is at z=0, rover spawns above and drops
    spawn_x = "0.0"
    spawn_y = "0.0"
    spawn_z = "0.5"  # Start above surface, gravity will settle it

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_description},
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_leo",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "leo_rover",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
        ],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    robot_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="robot_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            # "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/imu/data_raw@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
            "/ouster/points/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        ],
        remappings=[
            ("/ouster/points/points", "/ouster/points"),
        ],
        output="screen",
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        arguments=[
            "/camera_front/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/camera_front/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera_front/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
        ],
        output="screen",
    )

    camera_points_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_points_bridge",
        arguments=[
            "/camera_front/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="moon_yard",
                description=(
                    "World to load: moon_yard (flat) or "
                    "moon_yard_craters (with craters)."
                ),
            ),
            OpaqueFunction(function=_resolve_world),
            robot_state_publisher,
            spawn_robot,
            clock_bridge,
            robot_bridge,
            camera_bridge,
            camera_points_bridge,
        ]
    )
