# UK Lunabotics 2025-2026

Software repository for the University of Leicester UK Lunabotics team.

## Prerequisites

- **Ubuntu 22.04**
- **ROS 2 Humble** - [Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- **Gazebo Sim** (Fortress)
- **Required packages:**
  ```bash
  sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-image ros-humble-xacro
  ```

## Quick Start

### 1. Clone

```bash
git clone https://github.com/KJdotIO/innex1-rover.git
cd innex1-rover
```

### 2. Install Dependencies

```bash
rosdep install --from-paths src -y --ignore-src
```

### 3. Build

```bash
colcon build --symlink-install
source install/setup.bash
```

### 4. Launch Simulation

```bash
ros2 launch lunabot_simulation moon_yard.launch.py
```

You should see Gazebo open with our competition arena and a Leo Rover in the starting zone.

### 5. Drive the Robot

In a new terminal:

```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive the rover around using the keys provided.

## Package Structure

```
src/
├── external/                # Vendored third-party packages
│   ├── leo_common-ros2/     # Leo Rover description and messages
│   └── leo_simulator-ros2/  # Leo Gazebo simulation
├── lunabot_bringup/         # System launch files
├── lunabot_control/         # Control algorithms
├── lunabot_description/     # Robot models and meshes
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
