from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Odometry Node (Issue #59)
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[{'publish_tf': False}],
            remappings=[('/odom', '/visual_odometry')] # Remap to avoid wheel odom conflict
        ),

        # 2. SLAM Node (Issue #59)
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{'Grid/CellSize': '0.05'}],
            arguments=['-d'], # Issue #59: Delete database on startup for testing
            remappings=[('/odom', '/odometry/filtered')] # Subscribe to fused data
        )
    ])
