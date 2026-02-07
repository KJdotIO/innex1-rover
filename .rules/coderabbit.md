# Innex1 Rover - CodeRabbit Review Guidelines

Project:
University of Leicester Lunabotics 2026 lunar rover.

## Stack

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Ignition Fortress
- C++ / Python

## Documentation

Architecture and contracts: [https://github.com/KJdotIO/innex1-rover/wiki](https://github.com/KJdotIO/innex1-rover/wiki)

Review PRs for consistency with documented interfaces, but note that contracts may evolve - code can lead, docs can follow.

## Standards

- ROS 2 conventions: snake_case topics/nodes, CamelCase message types
- Python: PEP 8, docstrings on public functions
- C++: ROS 2 style guide
- `colcon build` should pass cleanly
- Appropriate QoS settings

## Focus Areas

- TF frame consistency
- QoS compatibility between pub/sub
- Error handling for sensor failures
- Sim/hardware abstraction (same interfaces, different drivers)
