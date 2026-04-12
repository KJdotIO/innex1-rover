# lunabot_localisation

This package contains localisation launch/config and localisation helper nodes for the rover stack.

## What this package is responsible for

`lunabot_localisation` provides fused localisation outputs consumed by planning and mapping components. It owns configuration around EKF-based local/global pose estimation and the bounded start-zone tag initialisation path.

## Core outputs

- Local fused odometry for navigation smoothness.
- Global pose initialisation and bounded stationary recovery through the start-zone tag path.
- TF relationships required by planning components.

## June baseline

The current baseline is deliberately simple: wheel odom, IMU, and a start-zone
AprilTag pose seed. The tag is used to initialise the global pose, not to
continuously correct the global EKF while the rover is driving. RTAB-Map can
still be launched for experimentation, but raw visual odometry is not fused
into the EKFs as part of the June baseline.

For backend testing, `launch/localisation.launch.py` now supports:

- `local_odometry_backend:=ekf` for the wheel-odom + IMU baseline
- `local_odometry_backend:=lio` for the Point-LIO path

The LIO backend uses a rover-specific sim config when `use_sim_time:=true`
and a hardware-oriented Ouster config otherwise.

## Key files

- `config/ekf.yaml`: EKF fusion and frame configuration.
- `launch/localisation.launch.py`: localisation bring-up path.
- `../external/point_lio/config/marsim.yaml`: Point-LIO sim config for the
  Gazebo Ouster bridge.
- `../external/point_lio/config/lunabot_ouster.yaml`: Point-LIO hardware
  placeholder config for the real Ouster path.
- `lunabot_localisation/tag_pose_publisher.py`: bridge from tag detections to bounded pose seeding / recovery inputs.

## Common failure modes

- Frame mismatch (`base_link` vs `base_footprint`) causing downstream planner instability.
- Covariances set unrealistically low, making the filter overconfident.
- Weak continuous odometry causing drift to accumulate after the start-zone seed.

## Where to read next

- Wiki: [Localisation](https://github.com/KJdotIO/innex1-rover/wiki/Localisation), [Extended Kalman Filtering Algorithm (EKF)](https://github.com/KJdotIO/innex1-rover/wiki/Extended-Kalman-Filtering-Algorithm-(EKF)), [Design-decisions](https://github.com/KJdotIO/innex1-rover/wiki/Design-decisions), [Contracts](https://github.com/KJdotIO/innex1-rover/wiki/Contracts)
- External references: [robot_localization package overview](https://index.ros.org/p/robot_localization/), [robot_localization GPS integration notes](https://docs.ros.org/en/noetic/api/robot_localization/html/integrating_gps.html), [REP-105](https://www.ros.org/reps/rep-0105.html), [Nav2 odometry setup guide](https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html)
