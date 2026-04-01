# INNEX1 Rover Software Stack (UK Lunabotics 2026)

This repository contains the University of Leicester rover software stack for UK Lunabotics 2026.

Linux is the primary runtime target for development and testing.

If you are on macOS or Windows, use an Ubuntu VM for realistic run and validation:
- macOS options: UTM, OrbStack, Parallels, VMware Fusion.
- Windows options: WSL2 (with GUI support where needed), Hyper-V, VirtualBox, VMware Workstation.

The key point is consistency: run the same Ubuntu + ROS + Gazebo stack as the team baseline.

## Baseline versions (team default)

These are the versions we target by default:
- OS: Ubuntu 22.04 LTS
- ROS: ROS 2 Humble
- Gazebo: Gazebo Fortress (`ignition-fortress` + `ros-humble-ros-gz`)

## Quick start (Linux, recommended path)

If you want the fastest working path, use the baseline above and run this sequence.

```bash
# 1) System deps
sudo apt update && sudo apt install -y locales curl gnupg2 lsb-release software-properties-common
sudo locale-gen en_GB en_GB.UTF-8
sudo update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8
export LANG=en_GB.UTF-8
sudo add-apt-repository -y universe

# 2) ROS 2 apt source
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3) Gazebo apt source
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 4) Install core stack
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-dev-tools \
  python3-rosdep \
  python3-pip \
  python3-venv \
  ros-humble-ros-gz \
  ros-humble-teleop-twist-keyboard \
  ignition-fortress \
  ignition-launch-cli

# 5) ROS environment
sudo rosdep init || true
rosdep update
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## OAK-D Pro on Humble

The hardware camera path targets the DepthAI v3 ROS driver on Humble.

Recommended setup:
- install `ros-$ROS_DISTRO-depthai-ros-v3`
- if you need the newest Humble builds before they hit stable apt, install `ros2-testing-apt-source` first
- make sure the OAK-D Pro has a good cable and enough power before blaming software
- keep the first bring-up on the `usb2_degraded` profile unless you know the USB 3 path is healthy

The wrapper launch added in this repo expects `depthai_ros_driver_v3` and its `driver.launch.py` entrypoint.

## Clone and build workspace

```bash
git clone https://github.com/KJdotIO/innex1-rover.git
cd innex1-rover

rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

Optional helper alias for GZ Web:

```bash
echo "alias gzweb='ign launch \$(ros2 pkg prefix lunabot_simulation)/share/lunabot_simulation/config/websocket.ign'" >> ~/.bashrc
source ~/.bashrc
```

## First run

### 1) Start simulation

```bash
source /opt/ros/humble/setup.bash
source ~/innex1-rover/install/setup.bash
ros2 launch lunabot_simulation moon_yard.launch.py
```

### 2) Start navigation stack (new terminal)

```bash
source /opt/ros/humble/setup.bash
source ~/innex1-rover/install/setup.bash
ros2 launch lunabot_bringup navigation.launch.py
```

### Hardware camera bring-up

Localisation-first hardware bring-up:

```bash
source /opt/ros/humble/setup.bash
source ~/innex1-rover/install/setup.bash
ros2 launch lunabot_bringup hardware_localisation.launch.py
```

That launch is not the whole robot. It brings up the OAK wrapper plus the localisation stack, and still expects the base robot drivers that provide `/odom`, `/imu/data_raw`, and the TF chain to be running elsewhere.

Full hardware navigation bring-up still expects the Ouster path to be present:

```bash
source /opt/ros/humble/setup.bash
source ~/innex1-rover/install/setup.bash
ros2 launch lunabot_bringup hardware_navigation.launch.py
```

Note on launch files:
- `navigation.launch.py` already includes `lunabot_bringup/launch/localisation.launch.py`.
- So for normal nav testing, launch **navigation only**.
- Launch `localisation.launch.py` separately only when you explicitly want localisation in isolation.
- Hardware launches default to `use_sim_time:=false` and keep `enable_visual_slam:=false`.

### 3) Manual teleop (new terminal)

```bash
source /opt/ros/humble/setup.bash
source ~/innex1-rover/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Health checks (`doctor`)

Use the built-in preflight checker before testing.

```bash
python3 tools/doctor.py                 # setup checks (default)
python3 tools/doctor.py --mode all      # setup + runtime (runtime auto-skips if stack not up)
python3 tools/doctor.py --mode runtime  # force runtime checks
```

Hardware camera readiness:

```bash
python3 tools/doctor.py --mode all \
  --config src/lunabot_bringup/config/preflight_checks_hardware_localisation.yaml
ros2 run lunabot_bringup preflight_check \
  --config src/lunabot_bringup/config/preflight_checks_hardware_localisation.yaml
python3 tools/check_oak_launch.py --profile usb2_degraded
```

The localisation hardware profile still expects the rover's main IMU on `/imu/data_raw`. OAK IMU fusion is deliberately not part of the first pass.
If you deliberately run `lidar_costmap_phase:=true`, use `preflight_checks_hardware_localisation_lidar_debug.yaml` instead.

Full hardware navigation readiness:

```bash
python3 tools/doctor.py --mode all \
  --config src/lunabot_bringup/config/preflight_checks_hardware_navigation.yaml
ros2 run lunabot_bringup preflight_check \
  --config src/lunabot_bringup/config/preflight_checks_hardware_navigation.yaml
```

If you deliberately run `lidar_costmap_phase:=true`, use `preflight_checks_hardware_navigation_lidar_debug.yaml` instead.

Exit codes:
- `0`: all checks passed
- `1`: warnings present
- `2`: failures present

## First hardware run checklist

Before you plug the camera in:
- run `python3 tools/check_oak_launch.py --profile usb2_degraded`
- if you want the full no-device smoke test as well, run `python3 tools/check_oak_launch.py --profile usb2_degraded --runtime`
- make sure the base robot stack is already publishing `/odom`, `/imu/data_raw`, and the usual TF chain

Once the OAK-D Pro is attached:
- launch `ros2 launch lunabot_bringup hardware_localisation.launch.py`
- expect `/camera_front/image` and `/camera_front/camera_info` first
- on USB 2, treat missing depth and point cloud as normal unless you’ve explicitly proved otherwise
- if the adapter warns that the required RGB streams are missing, check the upstream OAK topic names before assuming anything deeper is broken

Bad signs:
- `/camera_front/image` never appears
- `camera_contract_adapter` warns after startup that no RGB streams were seen
- frame names start leaking through as `oak_*` or `camera_link`

## Visualisation options

| Tool | Purpose | Command |
|---|---|---|
| RViz2 | ROS visualisation and TF/topics | `ros2 run rviz2 rviz2` |
| GZ Web | World/arena render | `gzweb` |
| Foxglove Bridge | Web visualisation bridge | `ros2 run foxglove_bridge foxglove_bridge` |

Typical endpoints:
- GZ Web: `ws://localhost:9002`
- Foxglove: `ws://localhost:8765`

## Common issues

If you get missing models on first launch, make sure the machine has internet once so Gazebo can populate cache (`~/.gz/fuel`).

If Gazebo is already running in a bad state, kill old instances and relaunch:

```bash
pkill -9 -f "gz sim"
```

If `hazard_detection` dies with `open3d` import errors, install it in your active Python environment.

## Repository layout

```text
src/
├── external/                # Vendored third-party packages
├── lunabot_bringup/         # Integrated launch entrypoints
├── lunabot_control/         # Motion/material action servers
├── lunabot_description/     # Robot model and meshes
├── lunabot_interfaces/      # Shared ROS action/message contracts
├── lunabot_localisation/    # EKF + global correction path
├── lunabot_navigation/      # Nav2 config + mission BT assets
├── lunabot_perception/      # Hazard detection pipeline
├── lunabot_sensors/         # Hardware sensor wrappers
├── lunabot_simulation/      # Gazebo world/bridge launch
└── lunabot_teleop/          # Manual control
```

## Interface contracts in CI

Interface contract checks run in CI via `.github/scripts/check_interface_contracts.py`.

Contract source of truth:
- `.github/contracts/interface_contracts.json`

If you rename a topic, action, or TF link, update the contract JSON in the same PR.

## Deeper docs

- Project wiki: https://github.com/KJdotIO/innex1-rover/wiki
- Architecture: https://github.com/KJdotIO/innex1-rover/wiki/SoftwareArchitecture
- Operations: https://github.com/KJdotIO/innex1-rover/wiki/Operations
- Contracts: https://github.com/KJdotIO/innex1-rover/wiki/Contracts

## Contributing and licence

- Contributing guide: [CONTRIBUTING.md](CONTRIBUTING.md)
- Licence: [Apache-2.0](LICENSE)

## Troubleshooting

**Launch file not found:**
Ensure launch and worlds directories are installed in `CMakeLists.txt`, then rebuild.

**Multiple Gazebo instances:**
Kill all: `pkill -9 -f "gz sim"`

**Models not loading:**
First download requires internet. Models cache locally in `~/.gz/fuel/` for offline use.

## Links

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Sim Documentation](https://gazebosim.org/docs)
- [Leo Rover Documentation](https://docs.fictionlab.pl/leo-rover)
- [UK Lunabotics Website](https://uklunabotics.co.uk/)
