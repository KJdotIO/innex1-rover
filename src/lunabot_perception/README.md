# lunabot_perception

This package contains perception nodes that convert sensor streams into navigation-relevant observations.

## What this package is responsible for

`lunabot_perception` turns front depth inputs into hazard observations that Nav2 costmaps can consume. It does not perform global planning and it does not own mission sequencing.

## Core responsibilities

- Process depth point clouds into terrain hazard outputs.
- Publish a local hazard grid and hazard marker points with stable topics and frames.
- Support tuning of crater and drop-off extraction for competition terrain.

## Key files

- `lunabot_perception/terrain_hazard_detector.py`: local crater hazard detector.
- `lunabot_perception/terrain_hazard_core.py`: pure helper logic for the detector.

## Common failure modes

- Missing runtime dependencies (for example `open3d` in environments still using the older implementation path).
- QoS mismatch with upstream camera topics.
- Frame mismatch causing hazards to appear in incorrect locations.

## Where to read next

- Wiki: [Perception](https://github.com/KJdotIO/innex1-rover/wiki/Perception), [Sensors](https://github.com/KJdotIO/innex1-rover/wiki/Sensors), [SoftwareArchitecture](https://github.com/KJdotIO/innex1-rover/wiki/SoftwareArchitecture), [Contracts](https://github.com/KJdotIO/innex1-rover/wiki/Contracts)
- External reference: [ROS 2 QoS overview](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
