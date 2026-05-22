# lunabot_localisation

Localisation launch, configuration, and helper nodes for the rover stack.

## Architecture

The localisation pipeline has two layers:

1. **Local EKF** (`robot_localization`): fuses wheel odometry + IMU to produce
  a smooth `odom -> base_footprint` transform and `/odometry/local`.
2. **Optional RTAB-Map SLAM** (`rtabmap_slam`): when
  `enable_visual_slam:=true`, subscribes to RGB-D images from the front depth
  camera and `/odometry/local`. It publishes the `map -> odom` correction via
  visual loop closure and AprilTag landmark constraints.

The competition default keeps `enable_visual_slam:=false` and publishes an
identity `map -> odom` transform. That avoids using arena wall features for
localisation while the local EKF, start-zone AprilTag gate, and odometry frame
remain available.

## Competition legality

The localisation path must be explainable to inspection judges. It uses wheel
odometry, IMU data, AprilTags and TF. Experimental visual SLAM is opt-in until
we can prove its inputs do not use arena-wall features. Future LiDAR-inertial
work may use the OS1-128 after wall-region filtering is in place.

It must not use GPS, compass heading, ultrasonic proximity sensing, touch
sensors for obstacle avoidance, arena walls as localisation features, or
uploaded obstacle locations.

## Start-zone localisation

The `start_zone_localiser` node spins in the start zone to acquire a stable
AprilTag lock. Once the lock is stable, it publishes a READY status so that
travel autonomy may begin. When visual SLAM is enabled for experiments,
RTAB-Map receives the same AprilTag detections as landmarks, aligning the map
frame with the known tag position.

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
- Wall points or simulation boundary geometry reaching a localisation backend
  that will later be claimed as competition-legal.

## Related docs

- Wiki: [Localisation](https://github.com/KJdotIO/innex1-rover/wiki/Localisation), [Design-decisions](https://github.com/KJdotIO/innex1-rover/wiki/Design-decisions)
- External: [RTAB-Map ROS2](https://github.com/introlab/rtabmap_ros), [robot_localization](https://index.ros.org/p/robot_localization/), [REP-105](https://www.ros.org/reps/rep-0105.html)
