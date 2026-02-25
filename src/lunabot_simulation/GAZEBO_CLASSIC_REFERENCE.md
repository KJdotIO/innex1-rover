# Gazebo Classic reference (Humble)

This branch uses Gazebo Classic via `gazebo_ros` and `gazebo_plugins`.

## Install commands

```bash
sudo apt update
sudo apt install -y \
  gazebo \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-plugins \
  ros-humble-gazebo-ros-pkgs
```

## Launch commands

```bash
ros2 launch lunabot_simulation moon_yard.launch.py
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity leo_rover
./src/lunabot_simulation/scripts/smoke_test_classic.sh
```

`moon_yard.launch.py` starts `gzserver` with:

```bash
gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so <world.sdf>
```

## Plugin mapping used in this migration

- Drive and odometry: `libgazebo_ros_diff_drive.so`
- Joint states: `libgazebo_ros_joint_state_publisher.so`
- IMU: `libgazebo_ros_imu_sensor.so`
- 2D lidar: `libgazebo_ros_ray_sensor.so`
- Camera/depth/points: `libgazebo_ros_camera.so`

## Topic targets for parity with the existing stack

- `/cmd_vel`
- `/odom`
- `/joint_states`
- `/imu/data_raw`
- `/scan`
- `/camera_front/image`
- `/camera_front/camera_info`
- `/camera_front/depth_image`
- `/camera_front/points`
- `/camera/image_raw`
- `/camera/camera_info`

`/camera/image_raw` is published by the base Leo camera plugin, so it is a quick Leo integration signal.

The smoke script also runs a movement check by publishing `/cmd_vel` and confirming `/odom` changes.

## Notes

- Gazebo Classic is EOL upstream. This branch is a practical fallback for local stability.
- The vendor package `src/external/leo_simulator-ros2` is disabled here using `COLCON_IGNORE` because it is Gazebo Sim / `ros_gz` specific.
