# lunabot_description

URDF/xacro robot model for the INNEX-1 rover.

## Overview

The model extends the stock Leo Rover description (`leo_description`) with
the sensor payloads used on the competition rover:

- **Ouster LiDAR** (`ouster_link`) — mounted on top of the base, 32-beam
  GPU LiDAR in simulation (1024 horizontal samples, 20 m range).
- **Front depth camera** (`camera_front_link` / `camera_front_optical_frame`)
  — 640 x 480 RGBD at 15 Hz, 90° FOV, tilted 0.3 rad downward.
- **Rear depth camera** (`camera_rear_link` / `camera_rear_optical_frame`)
  — mirrors the front camera, facing backward for reverse-drive obstacle
  detection.

Optical frames follow the REP-103 convention (Z forward, X right, Y down).

## Key files

- `urdf/lunabot.urdf.xacro`: the single xacro file. Includes Leo Rover base,
  adds sensor links/joints, and defines Gazebo sensor plugins.

## Usage

The xacro is processed at launch time by `robot_state_publisher`. In the
simulation launch (`lunabot_simulation`), it is loaded via:

```python
robot_description = xacro.process(
    str(pkg_lunabot_description / "urdf" / "lunabot.urdf.xacro")
)
```

## TF frames added

| Frame | Parent | Purpose |
|-------|--------|---------|
| `ouster_link` | `base_link` | LiDAR origin |
| `camera_front_link` | `base_link` | Front camera body |
| `camera_front_optical_frame` | `camera_front_link` | Optical convention |
| `camera_rear_link` | `base_link` | Rear camera body |
| `camera_rear_optical_frame` | `camera_rear_link` | Optical convention |
