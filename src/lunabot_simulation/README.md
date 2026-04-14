# lunabot_simulation

Gazebo Fortress simulation environment for the INNEX-1 rover, matching the
UK Lunabotics arena (7.9 m x 4.4 m).

## Worlds

| World file | Description |
|------------|-------------|
| `worlds/moon_yard.sdf` | Flat regolith surface with scattered rocks. |
| `worlds/moon_yard_craters.sdf` | Terrain with crater features for rougher traversal testing. |

Both worlds include arena boundary walls and an AprilTag beacon for
start-zone localisation.

## Launch

```bash
# Default flat world
ros2 launch lunabot_simulation moon_yard.launch.py

# Cratered variant
ros2 launch lunabot_simulation moon_yard.launch.py world:=moon_yard_craters
```

The launch file processes the `lunabot_description` xacro, starts
`robot_state_publisher`, spawns the rover via `ros_gz_sim`, and sets up
`ros_gz_bridge` nodes for the following topics:

### Bridged topics

| Gazebo topic | ROS 2 topic | Type |
|--------------|-------------|------|
| `cmd_vel` | `/cmd_vel_safe` | `geometry_msgs/Twist` |
| `odom` | `/odom` | `nav_msgs/Odometry` |
| `imu/data_raw` | `/imu/data_raw` | `sensor_msgs/Imu` |
| `joint_states` | `/joint_states` | `sensor_msgs/JointState` |
| `ouster/points` | `/ouster/points` | `sensor_msgs/PointCloud2` |
| `camera_front/*` | `/camera_front/{image,depth_image,camera_info,points}` | Image / CameraInfo / PointCloud2 |
| `camera_rear/*` | `/camera_rear/{image,depth_image,camera_info,points}` | Image / CameraInfo / PointCloud2 |
| `clock` | `/clock` | `rosgraph_msgs/Clock` |

## Key files

- `launch/moon_yard.launch.py`: main simulation launch.
- `worlds/`: SDF world files.
- `models/`: custom Gazebo models (rocks, arena walls, AprilTag).
- `config/`: bridge and plugin configuration.

## macOS fallback

On macOS the launch file automatically patches `ogre2` → `ogre` in the world
SDF (Ogre2 is unsupported on macOS Metal). The patched file is written to a
temp path and used transparently.

## Notes

- `GZ_SIM_RESOURCE_PATH` is prepended with the package's `models/` directory
  at launch so Gazebo can find custom models.
- The rover spawns at (0, 0, 0.5) and settles under gravity.
- `use_sim_time: true` is set on `robot_state_publisher`.
