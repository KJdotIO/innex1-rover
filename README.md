# UK Lunabotics 2025-2026

Software repository for the University of Leicester UK Lunabotics team.

## Prerequisites

- **Ubuntu 22.04**
- **ROS 2 Humble** - [Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- **Gazebo Sim** (Ignition Gazebo 6+)
- **ros_gz packages:**
  ```bash
  sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
  ```

## Quick Start

### 1. Clone

```bash
sudo apt install git
git clone https://github.com/tju2025/UK-Lunabotics-Software-Engineers-2025-2026.git
cd UK-Lunabotics-Software-Engineers-2025-2026
```

### 2. Build

```bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch Test World

```bash
ros2 launch lunabot_simulation test_world.launch.py
```
This may cause your system to lag, or gazebo to become unresponsive. This is due to the complexity of the model, so click wait and give it time to load.

You should see Gazebo open with a ground plane and ELM4 chassis model.

## Package Structure

```
src/
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
2. Kill Gazebo: `pkill -9 -f "ign gazebo"`
3. Relaunch: `ros2 launch lunabot_simulation test_world.launch.py`

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
Kill all: `pkill -9 -f "ign gazebo"`

**Models not loading:**  
First download requires internet. Models cache locally in `~/.gz/fuel/` for offline use.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for development guidelines.

## License

Licensed under Apache-2.0. See [LICENSE](LICENSE) file.

## Links

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Sim Documentation](https://gazebosim.org/docs)
- [UK Lunabotics Website](https://uklunabotics.co.uk/)
