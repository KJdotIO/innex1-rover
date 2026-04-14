# lunabot_bringup

This package contains launch entrypoints that start integrated stack configurations.

## What this package is responsible for

`lunabot_bringup` is the operational entrypoint package. It composes
localisation, navigation, safety (lunabot_safety estop_node), teleop, and control-related nodes into
runnable launch flows for testing and operation. The June baseline does not
launch a standalone perception package.

## Typical usage

Use bring-up launches when validating end-to-end behaviour. Avoid debugging subsystem-level issues from partial ad-hoc launches unless intentionally isolating one component.

## Key files

- `launch/`: stack launch entrypoints (navigation and related orchestration paths).
- `launch/mission_manager.launch.py`: starts the standalone mission supervisor node.
- `launch/mission_dry_run.launch.py`: one-command sim dry run for travel, excavate, and deposit.

## Mission Manager

Start just the mission supervisor node with:

```bash
ros2 launch lunabot_bringup mission_manager.launch.py
```

For direct executable discovery without the launch wrapper, run:

```bash
ros2 run lunabot_bringup mission_manager
```

This launch path only starts the supervisor node itself. Navigation, excavation,
and deposition servers remain in their existing bringup paths and are not
implicitly started here.

## Mission Dry Run

Use the dry-run launch when you want one bounded operator-facing regression pass in sim:

```bash
ros2 launch lunabot_bringup mission_dry_run.launch.py
```

The harness runs runtime preflight, then one travel attempt through `/navigate_to_pose_gate`,
one bounded excavation attempt through `/mission/excavate`, and one bounded deposit attempt
through `/mission/deposit`. It prints flat summary lines:

- `travel: pass/fail`
- `excavate: pass/fail`
- `deposit: pass/fail`
- `overall: pass/fail`

For a direct shell exit code without the composed sim launch, run the harness entrypoint:

```bash
ros2 run lunabot_bringup mission_dry_run
```

## Common failure modes

- Launch starts successfully but one critical node crashes shortly after (dependency mismatch).
- Lifecycle nodes stay unconfigured due to invalid params in one server config.
- Nodes appear alive but key topic links fail due to QoS incompatibility.

## Where to read next

- Wiki: [Operations](https://github.com/KJdotIO/innex1-rover/wiki/Operations), [SoftwareArchitecture](https://github.com/KJdotIO/innex1-rover/wiki/SoftwareArchitecture), [Autonomy-cycle-walkthrough](https://github.com/KJdotIO/innex1-rover/wiki/Autonomy-cycle-walkthrough), [Contracts](https://github.com/KJdotIO/innex1-rover/wiki/Contracts)
