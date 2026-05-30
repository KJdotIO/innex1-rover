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

## Jetson runtime profile

Before a scored-run rehearsal, check that the runtime/comms profile is still
lean:

```bash
ros2 run lunabot_bringup runtime_profile check
ros2 run lunabot_bringup runtime_profile show --profile hardware_competition
```

The profile rules live in
[`docs/jetson_runtime_profiles.md`](docs/jetson_runtime_profiles.md).

## Mission evidence bags

Use the evidence helper when a run should be reviewed later:

```bash
ros2 run lunabot_bringup mission_evidence \
  --profile minimal \
  --label golden-shuttle \
  --use-sim-time \
  -- ros2 launch lunabot_bringup mission_shuttle_evidence.launch.py
```

The workflow is documented in
[`docs/mission_evidence_workflow.md`](docs/mission_evidence_workflow.md).

## Hardware week runbook

For first-motion, ground-control, comms and evidence-capture procedure on the
Jetson, use [`docs/hardware_week_runbook.md`](docs/hardware_week_runbook.md).

For remote support after the lab handoff, use
[`docs/remote_competition_handoff.md`](docs/remote_competition_handoff.md).

## Active runtime paths

Use [`docs/active_runtime_paths.md`](docs/active_runtime_paths.md) to check
which launch files are current for sim, hardware bring-up, mission evidence,
and operator telemetry.

## Doctor and Docker

Run the doctor before setup, handoff, or hardware work:

```bash
python3 tools/doctor.py --profile developer
python3 tools/doctor.py --profile operator
python3 tools/doctor.py --profile jetson
```

For a repeatable ROS 2 Humble development shell, use
[`docs/docker_workflow.md`](docs/docker_workflow.md). Docker is for repo work and
cheap checks; hardware truth still comes from the Jetson and the rover.

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

## Repository layout

```text
src/
├── external/                # Vendored third-party packages (Leo Rover)
├── lunabot_bringup/         # Top-level launch files and mission manager
├── lunabot_control/         # Deposition bridge (Cytron linear actuators)
├── lunabot_description/     # URDF/xacro robot model and sensor frames
├── lunabot_drivetrain/      # Teensy/Sabertooth drivetrain bridge, velocity gate, stall detection
├── lunabot_excavation/      # Excavation action server
├── lunabot_interfaces/      # Custom ROS messages and actions
├── lunabot_localisation/    # EKF + RTAB-Map SLAM + AprilTag localisation
├── lunabot_navigation/      # Nav2 config, costmaps, collision monitor
├── lunabot_perception/      # Crater detection feeding Nav2 costmaps
├── lunabot_safety/          # E-stop to motion-inhibit bridge
├── lunabot_simulation/      # Gazebo Fortress worlds and ros_gz bridges
└── lunabot_teleop/          # Joystick and browser Gamepad manual control
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
- Local inspection packet: [docs/competition_inspection_packet.md](docs/competition_inspection_packet.md)
- Remote competition handoff: [docs/remote_competition_handoff.md](docs/remote_competition_handoff.md)
- Agent handoff: [docs/agent_handoff.md](docs/agent_handoff.md)
- Docker workflow: [docs/docker_workflow.md](docs/docker_workflow.md)

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
