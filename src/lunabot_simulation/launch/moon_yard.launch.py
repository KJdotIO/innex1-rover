import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_lunabot_description = get_package_share_directory("lunabot_description")
    pkg_lunabot_simulation = get_package_share_directory("lunabot_simulation")
    world_path = os.path.join(pkg_lunabot_simulation, "worlds", "moon_yard.sdf")

    # Set Gazebo Classic model/resource paths so custom models resolve.
    models_path = os.path.join(pkg_lunabot_simulation, "models")
    gazebo_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    gazebo_resource_path = os.environ.get("GAZEBO_RESOURCE_PATH", "")
    model_path_value = (
        models_path if not gazebo_model_path else f"{models_path}:{gazebo_model_path}"
    )
    resource_path_value = (
        models_path
        if not gazebo_resource_path
        else f"{models_path}:{gazebo_resource_path}"
    )

    robot_description = xacro.process(
        os.path.join(pkg_lunabot_description, "urdf", "lunabot.urdf.xacro")
    )

    # Spawn position - surface mesh is at z=0, rover spawns above and drops
    spawn_x = "0.0"
    spawn_y = "0.0"
    spawn_z = "0.5"  # Start above surface, gravity will settle it

    # Run Gazebo Classic server-only to keep resource usage low.
    gazebo_server = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            world_path,
        ],
        output="screen",
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
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_leo",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "leo_rover",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path_value),
            SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", resource_path_value),
            gazebo_server,
            robot_state_publisher,
            TimerAction(period=3.0, actions=[spawn_robot]),
        ]
    )
