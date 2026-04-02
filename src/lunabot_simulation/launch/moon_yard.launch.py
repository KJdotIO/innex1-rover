import os
import platform
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim").find("ros_gz_sim")
    pkg_lunabot_description = get_package_share_directory("lunabot_description")
    pkg_lunabot_simulation = get_package_share_directory("lunabot_simulation")
    world_path = os.path.join(pkg_lunabot_simulation, "worlds", "moon_yard.sdf")

    # switches to ogre 1 preflight if on mac. doesnt affect linux
    if platform.system() == "Darwin":
        with open(world_path, "r", encoding="utf-8") as world_file:
            content = world_file.read()
        if "<render_engine>ogre2</render_engine>" in content:
            patched_content = content.replace(
                "<render_engine>ogre2</render_engine>",
                "<render_engine>ogre</render_engine>",
            )
            tmp_world = os.path.join(tempfile.gettempdir(), "moon_yard_mac.sdf")
            with open(tmp_world, "w", encoding="utf-8") as world_file:
                world_file.write(patched_content)
            world_path = tmp_world
            print(f"macOS detected: Using patched Ogre1 world at {world_path}")

    # Set GZ_SIM_RESOURCE_PATH so Gazebo can find our custom models
    models_path = os.path.join(pkg_lunabot_simulation, "models")
    gz_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if models_path not in gz_resource_path:
        os.environ["GZ_SIM_RESOURCE_PATH"] = models_path + ":" + gz_resource_path

    robot_description = xacro.process(
        os.path.join(pkg_lunabot_description, "urdf", "lunabot.urdf.xacro")
    )

    world = LaunchConfiguration("world")
    headless_rendering = LaunchConfiguration("headless_rendering")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    # we'll run the sim without gazebo gui to save resources for now
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r -s '", world, "'"]}.items(),
        condition=UnlessCondition(headless_rendering),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r -s --headless-rendering '", world, "'"]}.items(),
        condition=IfCondition(headless_rendering),
    )

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
            "-Y",
            spawn_yaw,
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
                default_value=world_path,
                description="Absolute path to the Gazebo world file.",
            ),
            DeclareLaunchArgument(
                "headless_rendering",
                default_value="true" if platform.system() != "Darwin" else "false",
                description="Enable Gazebo EGL headless rendering for sensor cameras.",
            ),
            DeclareLaunchArgument(
                "spawn_x",
                default_value="0.0",
                description="Robot spawn x position in world coordinates.",
            ),
            DeclareLaunchArgument(
                "spawn_y",
                default_value="0.0",
                description="Robot spawn y position in world coordinates.",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.5",
                description="Robot spawn z position in world coordinates.",
            ),
            DeclareLaunchArgument(
                "spawn_yaw",
                default_value="0.0",
                description="Robot spawn yaw in radians.",
            ),
            SetEnvironmentVariable("DISPLAY", "", condition=IfCondition(headless_rendering)),
            gz_sim,
            gz_sim_headless,
            robot_state_publisher,
            spawn_robot,
            clock_bridge,
            robot_bridge,
            camera_bridge,
            camera_points_bridge,
        ]
    )
