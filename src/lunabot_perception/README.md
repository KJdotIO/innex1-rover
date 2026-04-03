# lunabot_perception

This package is a reserved shell for future perception nodes.

## What this package is responsible for

`lunabot_perception` is not part of the June runtime baseline. The current
stack uses direct sensor observation topics in Nav2 costmaps rather than a
standalone perception node in this package.

## Current status

- No runnable node is installed from this package today.
- No launch file depends on a baseline perception node here.
- The package stays in the repo as the future home for camera or depth-derived
  perception work once that runtime path is defined properly.

## Key files

- `package.xml`: package metadata.
- `setup.py`: installation metadata for the package shell.

## Common failure modes

- Assuming this package already contains a runnable hazard pipeline.
- Updating navigation or bring-up docs without checking whether a real
  perception node exists yet.

## Where to read next

- Wiki: [Perception](https://github.com/KJdotIO/innex1-rover/wiki/Perception), [Sensors](https://github.com/KJdotIO/innex1-rover/wiki/Sensors), [SoftwareArchitecture](https://github.com/KJdotIO/innex1-rover/wiki/SoftwareArchitecture), [Contracts](https://github.com/KJdotIO/innex1-rover/wiki/Contracts)
- External reference: [ROS 2 QoS overview](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
