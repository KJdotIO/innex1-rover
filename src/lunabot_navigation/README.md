# lunabot_navigation

`lunabot_navigation` holds the Nav2 configuration and behaviour tree assets used
by the rover navigation stack.

## What this package is responsible for

`lunabot_navigation` defines how Nav2 plans and follows paths in this project.
It does not estimate localisation and it does not detect hazards directly. It
consumes localisation outputs plus direct sensor observation topics used by the
costmaps.

## Inputs this package depends on

At runtime, navigation expects:
- fused odometry from localisation (`/odometry/filtered`),
- TF chain including `map`, `odom`, `base_footprint`,
- direct sensor observation sources used by costmaps,
- configured mission or operator navigation goals.

## Key files

- `config/nav2_params.yaml`: Nav2 planners, controller, costmaps, and BT navigator config.
- `behavior_trees/navigate_to_pose_bounded_recovery.xml`: active navigate-to-pose tree with bounded recovery.

## Current status

The runtime uses the bounded-recovery navigate-to-pose tree configured in
`nav2_params.yaml`. Higher-level mission sequencing lives outside Nav2 and
sends goals into this package through the standard navigation action path.

## Common failure modes

- BT XML path configured but file not installed into share directory.
- Costmaps not receiving expected observation topics.
- TF mismatch between localisation outputs and Nav2 frame config.
- Controller/planner parameters tuned for a different environment.

## Related docs

- Wiki: [SoftwareArchitecture](https://github.com/KJdotIO/innex1-rover/wiki/SoftwareArchitecture), [Planning](https://github.com/KJdotIO/innex1-rover/wiki/Planning), [StateManagement](https://github.com/KJdotIO/innex1-rover/wiki/StateManagement), [Design-decisions](https://github.com/KJdotIO/innex1-rover/wiki/Design-decisions), [Contracts](https://github.com/KJdotIO/innex1-rover/wiki/Contracts)
