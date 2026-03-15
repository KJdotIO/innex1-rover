import os
import platform
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a launch description for moon yard simulation.

    Includes Gazebo startup, rover spawn, state publishing, and bridge nodes.
    """
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim").find("ros_gz_sim")
    pkg_lunabot_description = get_package_share_directory("lunabot_description")
    pkg_lunabot_simulation = get_package_share_directory("lunabot_simulation")
    world_path = os.path.join(pkg_lunabot_simulation, "worlds", "moon_yard.sdf")

    # switches to ogre 1 preflight if on mac. doesnt affect linux
    if platform.system() == "Darwin":
        with open(world_path, "r") as f:
            content = f.read()
        if "<render_engine>ogre2</render_engine>" in content:
            patched_content = content.replace(
                "<render_engine>ogre2</render_engine>",
                "<render_engine>ogre</render_engine>",
            )
            tmp_world = os.path.join(tempfile.gettempdir(), "moon_yard_mac.sdf")
            with open(tmp_world, "w") as f:
                f.write(patched_content)
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

    # Spawn in the Starting Zone (top-left of arena, centered at -2.95, 1.1)
    spawn_x = "-2.95"
    spawn_y = "1.1"
    spawn_z = "0.3"

    # we'll run the sim without gazebo gui to save resources for now
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r -s '{world_path}'"}.items(),
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
        ],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    # Core robot bridge: cmd_vel, odom, IMU, joint_states, pose
    robot_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="robot_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/imu/data_raw@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
        ],
        output="screen",
    )

    # Split camera bridges: point cloud on its own process to avoid
    # contention with image streams (the main cause of point cloud starvation).
    camera_qos_overrides = {
        "qos_overrides./camera_front/camera_info.publisher.reliability": "best_effort",
        "qos_overrides./camera_front/camera_info.publisher.history": "keep_last",
        "qos_overrides./camera_front/camera_info.publisher.depth": 5,
        "qos_overrides./camera_front/image.publisher.reliability": "best_effort",
        "qos_overrides./camera_front/image.publisher.history": "keep_last",
        "qos_overrides./camera_front/image.publisher.depth": 5,
        "qos_overrides./camera_front/depth_image.publisher.reliability": "best_effort",
        "qos_overrides./camera_front/depth_image.publisher.history": "keep_last",
        "qos_overrides./camera_front/depth_image.publisher.depth": 5,
        "qos_overrides./camera_front/points.publisher.reliability": "best_effort",
        "qos_overrides./camera_front/points.publisher.history": "keep_last",
        "qos_overrides./camera_front/points.publisher.depth": 5,
    }

    camera_image_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_image_bridge",
        arguments=[
            "/camera_front/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/camera_front/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera_front/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
        ],
        parameters=[camera_qos_overrides],
        output="screen",
    )

    camera_pointcloud_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_pointcloud_bridge",
        arguments=[
            "/camera_front/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        ],
        parameters=[camera_qos_overrides],
        output="screen",
    )

    return LaunchDescription(
        [
            gz_sim,
            robot_state_publisher,
            spawn_robot,
            clock_bridge,
            robot_bridge,
            camera_image_bridge,
            camera_pointcloud_bridge,
        ]
    )
