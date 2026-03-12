# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Stack

- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS 2 Humble
- **Sim**: Gazebo Fortress (`ignition-fortress` + `ros-humble-ros-gz`)
- **Language**: Python (ROS nodes), CMake (interface packages)

## Common Commands

Always source ROS before running any `ros2` or `colcon` command:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

**Build:**
```bash
colcon build --symlink-install
# Build a single package:
colcon build --symlink-install --packages-select lunabot_mission
```

**Test:**
```bash
colcon test && colcon test-result --verbose
# Test a single package:
colcon test --packages-select lunabot_mission && colcon test-result --verbose
```

**Interface contract check (runs in CI):**
```bash
python3 .github/scripts/check_interface_contracts.py
```

**Preflight / health check:**
```bash
python3 tools/doctor.py               # setup checks
python3 tools/doctor.py --mode all    # setup + runtime
```

**Run the full stack:**
```bash
# Terminal 1 — simulation
ros2 launch lunabot_simulation moon_yard.launch.py

# Terminal 2 — navigation (already includes localisation)
ros2 launch lunabot_bringup navigation.launch.py

# Terminal 3 — teleop (optional)
# Remapped to /teleop/cmd_vel so cmd_vel_mux can arbitrate with Nav2
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/teleop/cmd_vel
```

## Architecture

The workspace lives under `src/`. All custom packages are prefixed `lunabot_`.

### Data flow (runtime)

```
Sim (Gazebo)
  └─ /odom, /imu/data_raw, /camera_front/points
        │
        ├─► lunabot_localisation
        │     dual EKF: local (odom→base_footprint) + global (map→odom)
        │     AprilTag pose corrections injected into global EKF only
        │     publishes /odometry/filtered, /tag_pose
        │
        ├─► lunabot_perception (hazard_detection node)
        │     open3d filtering of /camera_front/points
        │     publishes /hazards/front  →  Nav2 costmap obstacle layer
        │
        └─► lunabot_navigation (Nav2)
              BT: mission_navigate_to_pose_bt.xml
              planner: SmacPlannerHybrid (Reeds-Shepp)
              controller: RegulatedPurePursuitController
              calls action servers on /mission/excavate, /mission/deposit
```

### Mission orchestration

`lunabot_mission/readiness_gate.py` — exposes `/mission/check_readiness` (std_srvs/Trigger). Checks that all critical topics are live and EKF covariance is below threshold before autonomy starts.

The behavior tree (`lunabot_navigation/behavior_trees/mission_navigate_to_pose_bt.xml`) sequences:
1. Readiness gate (topic checks)
2. `NavigateToPose` → dig site
3. `/mission/excavate` action
4. `NavigateToPose` → dump site
5. `/mission/deposit` action
6. Repeat until failure

Action stubs (for sim/testing) live in `lunabot_control/material_action_server.py`. They simulate timed progress through phases and support forced-failure injection via the `force_failure_action` parameter.

### Interface contracts

Topic names, message types, and TF links are enforced in CI via:
- **Contract file**: `.github/contracts/interface_contracts.json`
- **Checker**: `.github/scripts/check_interface_contracts.py`

**If you rename a topic, action, or TF frame, update the contract JSON in the same PR.**

### TF tree

```
map
 └─ odom          (global EKF — jumps on AprilTag correction)
     └─ base_footprint  (local EKF — smooth, never jumps)
         └─ base_link   (xacro joint, fixed offset)
```

### Key topic contracts

| Topic | Type | Producer |
|---|---|---|
| `/hazards/front` | `sensor_msgs/PointCloud2` | `hazard_detection` |
| `/odometry/filtered` | `nav_msgs/Odometry` | local EKF |
| `/camera_front/points` | `sensor_msgs/PointCloud2` | Gazebo bridge |
| `/mission/excavate` | `lunabot_interfaces/action/Excavate` | `material_action_server` |
| `/mission/deposit` | `lunabot_interfaces/action/Deposit` | `material_action_server` |

### Action interfaces (`lunabot_interfaces`)

`Excavate.action`: goal has `mode`, `timeout_s`, `target_fill_fraction`. Feedback streams `phase` (PRECHECK→SPINUP→DIGGING→RETRACT), `fill_fraction_estimate`, `jam_detected`, `estop_active`.

`Deposit.action`: goal has `mode`, `timeout_s`, `dump_duration_s`. Feedback streams `phase` (PRECHECK→OPENING→RAISING→DUMPING→CLOSING), `door_open`, `bed_raised`.

Both result types carry `success`, `reason_code` (typed constants), and `failure_reason`.

## Conventions

- Python nodes use `rclpy`; all new nodes go in their package's inner module directory (e.g. `lunabot_mission/lunabot_mission/`).
- Entry points are declared in `setup.py` under `console_scripts`.
- `use_sim_time: true` is set throughout — do not add nodes that ignore this.
- Nav2 config lives in `lunabot_navigation/config/nav2_params.yaml`; the BT XML path is wired via `bt_navigator.default_nav_to_pose_bt_xml`.
- CI runs flake8, pep257, and copyright checks on all Python packages (via the standard `colcon test` ament linters).
