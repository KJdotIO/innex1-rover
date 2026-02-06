from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1. Odometry Node
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[{'publish_tf': False}],
            remappings=[('/odom', '/visual_odometry')]
        ),

        # 2. SLAM Node
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{'Grid/CellSize': '0.05'}],
            arguments=['-d'],
            remappings=[('/odom', '/odometry/filtered')]
        )
    ])
