# INNEX1 Rover Software Stack (UK Lunabotics 2026)

This repository contains the University of Leicester rover software stack for UK Lunabotics 2026.

Linux is the primary runtime target for development and testing.

If you are on macOS or Windows, use an Ubuntu VM for realistic run and validation:
- macOS options: UTM, OrbStack, Parallels, VMware Fusion.
- Windows options: WSL2 (with GUI support where needed), Hyper-V, VirtualBox, VMware Workstation.

The key point is consistency: run the same Ubuntu + ROS + Gazebo stack as the team baseline.

For the team, the preferred shared workflow is now the Docker-based sim environment on Linux or a Linux cloud VM. That gives everyone the same ROS and Gazebo stack without rebuilding machines from scratch.

## Baseline versions (team default)

These are the versions we target by default:
- OS: Ubuntu 22.04 LTS
- ROS: ROS 2 Humble
- Gazebo: Gazebo Fortress (`ignition-fortress` + `ros-humble-ros-gz`)

## Quick start (Docker, recommended shared path)

If you want the most repeatable team workflow, use the container setup. This is the path we want people to converge on for simulation and remote work.

Detailed container notes live in [docs/development/containers.md](docs/development/containers.md).

### 1) Linux VM or cloud GPU host prerequisites

You need:

- Ubuntu 22.04
- Docker Engine
- NVIDIA container toolkit if the machine has an NVIDIA GPU

Check the GPU path with:

```bash
nvidia-smi
docker run --rm --gpus all nvidia/cuda:12.3.2-base-ubuntu22.04 nvidia-smi
```

### 2) Build the image

```bash
git clone https://github.com/KJdotIO/innex1-rover.git
cd innex1-rover
./docker/scripts/build_sim_image.sh
```

If Docker still needs `sudo` on your machine, that is acceptable:

```bash
sudo ./docker/scripts/build_sim_image.sh
```

The build script uses plain `docker build` by default because it is the least surprising option on shared Linux machines. If you explicitly want `buildx`, for example on a cross-build host, use:

```bash
USE_BUILDX=true ./docker/scripts/build_sim_image.sh
```

### 3) Build the workspace once inside the container

```bash
./docker/scripts/run_sim_dev.sh
colcon build --symlink-install
source install/setup.bash
exit
```

### 4) Start the stack

Run each command from the repo root in its own terminal:

```bash
./docker/scripts/run_sim_launch.sh
```

```bash
./docker/scripts/run_navigation_launch.sh
```

```bash
./docker/scripts/run_foxglove_bridge.sh
```

On a cloud VM, it is usually more practical to leave them running in the background:

```bash
DETACH=true ./docker/scripts/run_sim_launch.sh
DETACH=true ./docker/scripts/run_navigation_launch.sh
DETACH=true ./docker/scripts/run_foxglove_bridge.sh
```

The sim launcher defaults to `HEADLESS_RENDERING=true` and `SERVER_ONLY=true` in the container workflow. That matches the intended cloud path: Gazebo runs on the VM, Foxglove runs in your browser, and you do not rely on forwarded desktop graphics.

You can turn those off explicitly:

```bash
HEADLESS_RENDERING=false SERVER_ONLY=false ./docker/scripts/run_sim_launch.sh
```

That only changes Gazebo's launch mode. The current Docker helpers do not yet wire a desktop display into the container, so headless plus Foxglove is still the supported team path.

### 5) Connect Foxglove

From your browser:

- on the same machine: `ws://localhost:8765`
- from another machine: `ws://<vm-ip>:8765`

If your cloud firewall or AWS security group does not expose port `8765`, tunnel it over SSH instead:

```bash
ssh -L 8765:localhost:8765 -i /path/to/key.pem ubuntu@<vm-ip>
```

Then connect your local browser to `ws://localhost:8765`.

Foxglove should be the default remote visualisation path for the team. It is lighter and less annoying than trying to make full desktop visualisation work everywhere.

On the current NVIDIA cloud path, Gazebo headless rendering is good enough for point clouds and the navigation stack. If you need camera-image debugging, the safer route is still a VM desktop or DCV session outside this first-pass container path.

If you are already in Amazon DCV or another remote desktop, that is useful for occasional RViz2 or Gazebo GUI sessions. It should not be the only workflow.

To stop the named containers cleanly:

```bash
docker stop innex1-foxglove innex1-navigation innex1-sim
```

If your machine still requires `sudo` for Docker:

```bash
sudo docker stop innex1-foxglove innex1-navigation innex1-sim
```

To inspect logs from the running services:

```bash
docker logs innex1-sim --tail 100
docker logs innex1-navigation --tail 100
docker logs innex1-foxglove --tail 100
```

If your host still needs `sudo` for Docker, use `sudo docker logs ...` instead.

## Quick start (Linux, manual install path)

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

On a Linux GPU host you can enable Gazebo's official EGL headless path explicitly:

```bash
ros2 launch lunabot_simulation moon_yard.launch.py headless_rendering:=true server_only:=true
```

If you want to remove Gazebo's server-only flag:

```bash
ros2 launch lunabot_simulation moon_yard.launch.py headless_rendering:=false server_only:=false
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

For shared remote work, Foxglove is the preferred default. RViz2 is still useful, but better treated as an interactive desktop tool on a Linux box or DCV session rather than the main remote workflow.

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
