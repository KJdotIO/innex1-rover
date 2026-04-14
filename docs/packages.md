# Package Map

## Ownership — who owns what

| Package | Build | Owns |
|---|---|---|
| `lunabot_interfaces` | ament_cmake (ROSIDL) | All shared ROS contracts — actions, msgs, srvs. Edit here for new interfaces. |
| `lunabot_bringup` | ament_python | Top-level launch, mission execution, preflight system, dry-run harness |
| `lunabot_control` | ament_python | `material_action_server`, `material_action_client` — material movement |
| `lunabot_excavation` | ament_python | `excavation_action_server`, `excavation_controller`, sim proxy, telemetry mock |
| `lunabot_localisation` | ament_python | `start_zone_localiser`, `stereo_camera_info_publisher`, `tag_pose_publisher` |
| `lunabot_navigation` | ament_python | Nav2 params, behavior trees — does NOT contain navigation nodes themselves |
| `lunabot_description` | ament_cmake | URDF/Xacro assets only — robot model |
| `lunabot_simulation` | ament_cmake | Gazebo worlds (`moon_yard.sdf`), sim launch |
| `lunabot_teleop` | ament_python | Joystick control |
| `lunabot_perception` | ament_python | Placeholder — nothing implemented yet |
| `lunabot_safety` | ament_python | `estop_node` — subscribes `/safety/estop`, publishes `/safety/motion_inhibit` |

## External (vendored)
- `src/external/leo_common-ros2` — Leo Rover base packages
- `src/external/leo_simulator-ros2` — Leo Rover sim packages

## Key files per package

### lunabot_bringup
- `config/preflight_checks.yaml` — navigation stack preflight profile
- `config/preflight_checks_dry_run.yaml` — stricter dry-run profile
- `lunabot_bringup/mission_dry_run.py` — integration harness node
- `launch/mission_dry_run.launch.py` — simulation + full stack + harness

### lunabot_interfaces
- `action/Excavate.action`, `action/Deposit.action` — mission action definitions
- `CMakeLists.txt` — ROSIDL generation block

### lunabot_localisation
- `config/ekf.yaml` — EKF tuning parameters

### lunabot_navigation
- `config/` — Nav2 parameter files and behavior trees

### lunabot_description
- `urdf/lunabot.urdf.xacro` — full robot model

### lunabot_simulation
- `worlds/moon_yard.sdf` — primary test environment
