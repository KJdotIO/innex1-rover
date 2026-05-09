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

### Bounded retries and cycle limits

The mission manager wraps each action call in `_retry_action` with configurable
retry counts:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `max_nav_retries` | 2 | Extra attempts for each navigation goal |
| `max_excavation_retries` | 1 | Extra attempts for excavation |
| `max_deposition_retries` | 1 | Extra attempts for deposition |
| `max_shuttle_cycles` | 10 | Total excavate→deposit round-trips before halt |

If all retries are exhausted the manager transitions to a terminal failure state
rather than looping indefinitely.

### Safety gating

The mission manager subscribes to `/safety/estop` and `/safety/motion_inhibit`
(transient-local QoS). Before every action attempt, `_is_safe()` checks both
signals — if either is active the attempt is short-circuited with a safety-stop
failure, and no action goal is sent.

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

## Mission Evidence Packs

Use `mission_evidence` when you want a repeatable rosbag capture with the
run context kept next to it. The default profile is intentionally lean: mission
state, safety state, subsystem status, TF and command topics. Raw images and
point clouds only live in the `heavy` profile so normal dry runs do not quietly
turn into a disk and bandwidth test.

Create a dry-run evidence pack and record around a mission command:

```bash
ros2 run lunabot_bringup mission_evidence \
  --profile minimal \
  --label sim-dry-run \
  --use-sim-time \
  -- ros2 launch lunabot_bringup mission_dry_run.launch.py
```

The helper creates `~/innex1_mission_evidence/<timestamp>_<label>/` with:

- `bag/`: the rosbag2 recording.
- `logs/rosbag_record.log`: recorder output.
- `logs/mission_command.log`: mission command output when one is provided.
- `config/`: copied bring-up configs used to understand the run later.
- `manifest.json`: git SHA, profile, topic allowlist, commands and exit codes.

For a command preview without starting `ros2 bag record`, run:

```bash
ros2 run lunabot_bringup mission_evidence --plan-only --profile debug
```

Replay a saved pack with the command written in `manifest.json`, normally:

```bash
ros2 bag play ~/innex1_mission_evidence/<pack>/bag
```

The helper passes `--max-bag-duration` and `--max-bag-size` to rosbag2 so long
runs split cleanly instead of producing one awkward file. On the Jetson Humble
image the available storage backend is `sqlite3`; keep that as the default
unless `ros2 bag record --help` shows MCAP has been installed.

## Common failure modes

- Launch starts successfully but one critical node crashes shortly after (dependency mismatch).
- Lifecycle nodes stay unconfigured due to invalid params in one server config.
- Nodes appear alive but key topic links fail due to QoS incompatibility.

## Where to read next

- Wiki: [Operations](https://github.com/KJdotIO/innex1-rover/wiki/Operations), [SoftwareArchitecture](https://github.com/KJdotIO/innex1-rover/wiki/SoftwareArchitecture), [Autonomy-cycle-walkthrough](https://github.com/KJdotIO/innex1-rover/wiki/Autonomy-cycle-walkthrough), [Contracts](https://github.com/KJdotIO/innex1-rover/wiki/Contracts)
