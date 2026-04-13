# lunabot_localisation

Localisation launch, configuration, and helper nodes for the rover stack.

## Architecture

The localisation pipeline has two layers:

1. **Local EKF** (`robot_localization`): fuses wheel odometry + IMU to produce
  a smooth `odom -> base_footprint` transform and `/odometry/local`.
2. **RTAB-Map SLAM** (`rtabmap_slam`): subscribes to RGB-D images from the
  front depth camera and `/odometry/local`. Publishes the `map -> odom`
  correction via visual loop closure and AprilTag landmark constraints.

The global EKF has been removed. RTAB-Map handles drift correction directly
through its pose graph optimiser, which is the same pattern used by multiple
Lunabotics teams (College of DuPage, Chicago Robotics/EDT).

## Start-zone localisation

The `start_zone_localiser` node spins in the start zone to acquire a stable
AprilTag lock. Once the lock is stable, it publishes a READY status so that
travel autonomy may begin. RTAB-Map receives the same AprilTag detections as
landmarks, aligning the map frame with the known tag position.

## Key files

- `config/ekf.yaml`: local EKF fusion configuration.
- `config/rtabmap.yaml`: RTAB-Map SLAM parameters.
- `launch/localisation.launch.py`: localisation bring-up.
- `lunabot_localisation/tag_pose_publisher.py`: tag detection to pose bridge.
- `lunabot_localisation/start_zone_localiser.py`: tag search and readiness gate.

## Common failure modes

- Frame mismatch (`base_link` vs `base_footprint`) causing downstream planner instability.
- Covariances set unrealistically low, making the filter overconfident.
- RTAB-Map not receiving depth images (check topic names and QoS).
- Missing `map -> tag36h11:0` static TF preventing landmark alignment.

## Where to read next

- Wiki: [Localisation](https://github.com/KJdotIO/innex1-rover/wiki/Localisation), [Design-decisions](https://github.com/KJdotIO/innex1-rover/wiki/Design-decisions)
- External: [RTAB-Map ROS2](https://github.com/introlab/rtabmap_ros), [robot_localization](https://index.ros.org/p/robot_localization/), [REP-105](https://www.ros.org/reps/rep-0105.html)

