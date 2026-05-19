"""Launch the real Ouster LiDAR for local bring-up checks."""

import lifecycle_msgs.msg
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start the official Ouster driver with INNEX-1 topic and frame names."""
    sensor_hostname = LaunchConfiguration("sensor_hostname")
    udp_dest = LaunchConfiguration("udp_dest")
    lidar_mode = LaunchConfiguration("lidar_mode")
    timestamp_mode = LaunchConfiguration("timestamp_mode")
    proc_mask = LaunchConfiguration("proc_mask")
    point_type = LaunchConfiguration("point_type")
    v_reduction = LaunchConfiguration("v_reduction")
    min_range = LaunchConfiguration("min_range")
    max_range = LaunchConfiguration("max_range")

    driver = LifecycleNode(
        package="ouster_ros",
        executable="os_driver",
        namespace="ouster",
        name="os_driver",
        output="screen",
        parameters=[
            {
                "sensor_hostname": sensor_hostname,
                "udp_dest": udp_dest,
                "lidar_mode": lidar_mode,
                "timestamp_mode": timestamp_mode,
                "sensor_frame": "ouster_link",
                "lidar_frame": "ouster_link",
                "imu_frame": "ouster_link",
                "point_cloud_frame": "ouster_link",
                "pub_static_tf": False,
                "proc_mask": proc_mask,
                "point_type": point_type,
                "v_reduction": ParameterValue(v_reduction, value_type=int),
                "min_range": ParameterValue(min_range, value_type=float),
                "max_range": ParameterValue(max_range, value_type=float),
                "attempt_reconnect": True,
                "dormant_period_between_reconnects": 1.0,
                "use_system_default_qos": False,
            }
        ],
    )

    configure_driver = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_driver = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=driver,
            goal_state="inactive",
            entities=[
                LogInfo(msg="Ouster driver configured; activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(driver),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
            handle_once=True,
        )
    )

    fail_if_unreachable = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=driver,
            goal_state="finalized",
            entities=[LogInfo(msg="Ouster driver could not reach the sensor.")],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensor_hostname",
                description="Ouster sensor hostname or IP address.",
            ),
            DeclareLaunchArgument(
                "udp_dest",
                description="Jetson Ethernet IP address that receives LiDAR UDP packets.",
            ),
            DeclareLaunchArgument(
                "lidar_mode",
                default_value="512x10",
                description="Low-rate mode for first Jetson room checks.",
            ),
            DeclareLaunchArgument(
                "timestamp_mode",
                default_value="TIME_FROM_ROS_TIME",
                description="Use ROS receive time until hardware time sync is configured.",
            ),
            DeclareLaunchArgument(
                "proc_mask",
                default_value="IMU|PCL|SCAN|TLM",
                description="Publish IMU, point cloud, LaserScan and telemetry.",
            ),
            DeclareLaunchArgument(
                "point_type",
                default_value="xyz",
                description="Keep the debug cloud light; use xyzi/xyzir when intensity is needed.",
            ),
            DeclareLaunchArgument(
                "v_reduction",
                default_value="2",
                description="Drop every other vertical beam for first visualisation tests.",
            ),
            DeclareLaunchArgument(
                "min_range",
                default_value="0.25",
                description="Reject very close returns from mounts and hands.",
            ),
            DeclareLaunchArgument(
                "max_range",
                default_value="15.0",
                description="Room-debug range limit in metres.",
            ),
            driver,
            configure_driver,
            activate_driver,
            fail_if_unreachable,
        ]
    )
