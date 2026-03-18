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

Note on launch files:
- `navigation.launch.py` already includes `lunabot_bringup/launch/localisation.launch.py`.
- So for normal nav testing, launch **navigation only**.
- Launch `localisation.launch.py` separately only when you explicitly want localisation in isolation.

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

Exit codes:
- `0`: all checks passed
- `1`: warnings present
- `2`: failures present

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
