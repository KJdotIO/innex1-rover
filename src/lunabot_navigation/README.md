# lunabot_navigation

This package contains Nav2 configuration and mission navigation behaviour tree assets used by the rover navigation stack.

## What this package is responsible for

`lunabot_navigation` defines how Nav2 plans and follows paths in this project.
It does not estimate localisation and it does not detect hazards directly. In
the June baseline it consumes localisation outputs plus direct sensor
observation topics used by the costmaps.

## Inputs this package depends on

At runtime, navigation expects:
- fused odometry from localisation (`/odometry/filtered`),
- TF chain including `map`, `odom`, `base_footprint`,
- direct sensor observation sources used by costmaps,
- configured mission or operator navigation goals.

## Key files

- `config/nav2_params.yaml`: Nav2 planners, controller, costmaps, and BT navigator config.
- `behavior_trees/mission_navigate_to_pose_bt.xml`: current mission BT shell used for scaffolded orchestration.

## Current status

The BT configuration in this package is a shell intended for incremental development. It is designed to be safe, readable, and easy to extend in follow-up PRs.

## Common failure modes

- BT XML path configured but file not installed into share directory.
- Costmaps not receiving expected observation topics.
- TF mismatch between localisation outputs and Nav2 frame config.
- Controller/planner parameters tuned for a different environment.

## Where to read next

- Wiki: [SoftwareArchitecture](https://github.com/KJdotIO/innex1-rover/wiki/SoftwareArchitecture), [Planning](https://github.com/KJdotIO/innex1-rover/wiki/Planning), [StateManagement](https://github.com/KJdotIO/innex1-rover/wiki/StateManagement), [Design-decisions](https://github.com/KJdotIO/innex1-rover/wiki/Design-decisions), [Contracts](https://github.com/KJdotIO/innex1-rover/wiki/Contracts)
- Issue tracker epic for BT orchestration: https://github.com/KJdotIO/innex1-rover/issues/107
