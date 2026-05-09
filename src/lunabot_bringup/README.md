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
- `launch/mission_shuttle_evidence.launch.py`: one-cycle waypoint shuttle run for
  golden rosbag evidence.
- `launch/rover_diagnostics.launch.py`: publishes standard `/diagnostics`
  summaries for operator and bag review.

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

### Passive operator telemetry

The mission manager publishes a small Foxglove-friendly state contract:

| Topic | Type | Meaning |
|-------|------|---------|
| `/mission/state` | `std_msgs/String` | Current FSM state, lower-case |
| `/mission/autonomy_mode` | `std_msgs/String` | `idle`, `autonomous`, `motion_inhibited`, `estop`, `safe_fail`, or `halted` |
| `/mission/time_remaining_s` | `std_msgs/Float32` | Remaining time from the 1200 s mission budget |
| `/mission/cycle_count` | `std_msgs/Int32` | Completed excavation/deposition cycles |
| `/mission/last_failure_reason` | `std_msgs/String` | Last terminal or safety failure reason |

These topics are passive telemetry only. They are safe to show in Foxglove
during autonomy because they do not provide a command path back into the rover.

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

## Hardware Fault Injection

Use the hardware fault injector when you want software-only checks for real
hardware boundary failures. It publishes the same ROS interfaces as the
hardware-facing nodes, so preflight and dry-run tooling see a controller-offline
drivetrain, asserted E-stop, excavation driver fault, or lost localisation as a
real contract failure rather than a private test stub.

List the available scenarios:

```bash
ros2 run lunabot_bringup hardware_fault_injector --scenario nominal --list
```

Publish a healthy mock boundary for a short preflight smoke check:

```bash
ros2 run lunabot_bringup hardware_fault_injector \
  --scenario nominal --duration-s 8
```

In another terminal, check that the hardware-boundary preflight accepts the
healthy case:

```bash
ros2 run lunabot_bringup preflight_check \
  --config src/lunabot_bringup/config/preflight_checks_fault_injection.yaml \
  --use-sim-time false
```

Then publish a fault, for example:

```bash
ros2 run lunabot_bringup hardware_fault_injector \
  --scenario drivetrain_controller_offline --duration-s 12
```

The same preflight command should fail with a field mismatch such as
`fault_code expected=0 got=3`. That is the point: the failure is visible on the
real project topic and the operator-facing check says what changed.

Scenarios that publish `/safety/estop` or `/safety/motion_inhibit` require an
explicit `--allow-live-topics` flag. Use that only with an isolated
`ROS_DOMAIN_ID`; those topics are live safety inputs, not harmless log lines.

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

Record the one-cycle checkpoint shuttle route with:

This launch is simulation-only. It disables the AprilTag readiness gate and uses
sim-time defaults, so do not use it as a hardware bring-up path.

```bash
ros2 run lunabot_bringup mission_evidence \
  --profile minimal \
  --label golden-shuttle \
  --use-sim-time \
  -- ros2 launch lunabot_bringup mission_shuttle_evidence.launch.py \
    launch_rviz:=false \
    max_shuttle_cycles:=1
```

Replay a saved pack with the command written in `manifest.json`, normally:

```bash
ros2 bag play ~/innex1_mission_evidence/<pack>/bag
```

The helper passes `--max-bag-duration` and `--max-bag-size` to rosbag2 so long
runs split cleanly instead of producing one awkward file. On the Jetson Humble
image the available storage backend is `sqlite3`; keep that as the default
unless `ros2 bag record --help` shows MCAP has been installed.

## Runtime Diagnostics

Start the diagnostics aggregator with:

```bash
ros2 launch lunabot_bringup rover_diagnostics.launch.py
```

It publishes `diagnostic_msgs/DiagnosticArray` on `/diagnostics` for:

- safety command state
- drivetrain status
- start-zone localisation readiness
- excavation status

Each item uses standard `OK`, `WARN`, `ERROR`, and `STALE` levels so Foxglove,
bags, and future preflight tooling can show health without scraping logs.

## Common failure modes

- Launch starts successfully but one critical node crashes shortly after (dependency mismatch).
- Lifecycle nodes stay unconfigured due to invalid params in one server config.
- Nodes appear alive but key topic links fail due to QoS incompatibility.

## Where to read next

- Wiki: [Operations](https://github.com/KJdotIO/innex1-rover/wiki/Operations), [SoftwareArchitecture](https://github.com/KJdotIO/innex1-rover/wiki/SoftwareArchitecture), [Autonomy-cycle-walkthrough](https://github.com/KJdotIO/innex1-rover/wiki/Autonomy-cycle-walkthrough), [Contracts](https://github.com/KJdotIO/innex1-rover/wiki/Contracts)
