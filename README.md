# UK Lunabotics 2025-2026

Software repository for the University of Leicester UK Lunabotics team.

## Installation

### Ubuntu 22.04 (Native or OrbStack)

Run these commands to install ROS 2 Humble, Gazebo Fortress, and required dependencies:

```bash
sudo apt update && sudo apt install -y locales curl gnupg2 lsb-release
sudo locale-gen en_GB en_GB.UTF-8
sudo update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8
export LANG=en_GB.UTF-8
sudo add-apt-repository -y universe

# Add ROS 2 repository
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Add Gazebo (OSRF) repository
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-dev-tools \
  python3-rosdep \
  ros-humble-ros-gz \
  ros-humble-teleop-twist-keyboard \
  ignition-fortress \
  ignition-launch-cli
sudo rosdep init || true && rosdep update
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Orbstack with Foxglove (Optional)

OrbStack is recommended for macOS users to minimise performance overhead compared to traditional VMs. If you intend to use Foxglove Studio for visualisation, install the bridge:

```bash
sudo apt update && sudo apt install -y ros-humble-foxglove-bridge
```

1. Install [OrbStack](https://orbstack.dev/).
2. Create an Ubuntu 22.04 machine: `orb create ubuntu:22.04 rover-sim`
3. Enter the machine: `orb shell rover-sim`
4. Execute the installation commands listed above.

## Workspace Setup

```bash
git clone https://github.com/KJdotIO/innex1-rover.git
cd innex1-rover
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash

# Add shortcut for GZ Web
echo "alias gzweb='ign launch \$(ros2 pkg prefix lunabot_simulation)/share/lunabot_simulation/config/websocket.ign'" >> ~/.bashrc
source ~/.bashrc
```

## Running the Simulation

The simulation runs in headless mode by default to conserve resources.

### Launch Simulation

```bash
ros2 launch lunabot_simulation moon_yard.launch.py
```

### Visualisation

| Tool | Purpose | Command |
| :--- | :--- | :--- |
| Rviz2 | Standard ROS visualisation | `ros2 run rviz2 rviz2` |
| GZ Web | Arena geometry and world layout | `gzweb` |
| Foxglove Studio | Sensor data and robot state | `ros2 run foxglove_bridge foxglove_bridge` |

For GZ Web, open https://app.gazebosim.org/visualization and connect to `ws://localhost:9002`.
For Foxglove Studio, connect to `ws://localhost:8765`.

### Manual Control

In a new terminal:

```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Package Structure

```
src/
├── external/                # Vendored third-party packages
│   ├── leo_common-ros2/     # Leo Rover description and messages
│   └── leo_simulator-ros2/  # Leo Gazebo simulation
├── lunabot_bringup/         # System launch files
├── lunabot_control/         # Control algorithms
├── lunabot_description/     # Robot models and meshes
├── lunabot_interfaces/      # Shared ROS action/message contracts
├── lunabot_localisation/    # Localisation and mapping
├── lunabot_navigation/      # Path planning
├── lunabot_perception/      # Computer vision
├── lunabot_simulation/      # Gazebo worlds and launch files
└── lunabot_teleop/          # Manual control
```

## Development

### Editing World/Launch Files

With symlink install, you can edit `.sdf` and `.launch.py` files without rebuilding:

1. Edit the file
2. Kill Gazebo: `pkill -9 -f "gz sim"`
3. Relaunch: `ros2 launch lunabot_simulation moon_yard.launch.py`

### When to Rebuild

Rebuild only when changing:

- `CMakeLists.txt` or `package.xml`
- C++ source files
- Adding new files

```bash
colcon build --symlink-install
source install/setup.bash
```

## Troubleshooting

**Launch file not found:**
Ensure launch and worlds directories are installed in `CMakeLists.txt`, then rebuild.

**Multiple Gazebo instances:**
Kill all: `pkill -9 -f "gz sim"`

**Models not loading:**
First download requires internet. Models cache locally in `~/.gz/fuel/` for offline use.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for development guidelines.

## License

Licensed under Apache-2.0. See [LICENSE](LICENSE) file.

## Links

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Sim Documentation](https://gazebosim.org/docs)
- [Leo Rover Documentation](https://docs.fictionlab.pl/leo-rover)
- [UK Lunabotics Website](https://uklunabotics.co.uk/)
