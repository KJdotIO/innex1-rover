# Material Handling Interface Notes

This package contains mechanism-agnostic action contracts for excavation and deposition.

## Actions

- `/mission/excavate` uses `lunabot_interfaces/action/Excavate`
- `/mission/deposit` uses `lunabot_interfaces/action/Deposit`

## Action Semantics

- `mode` and `timeout_s` are mission intent fields.
- `target_fill_fraction`, `max_drive_speed_mps`, `dump_duration_s`, and `require_close_after_dump` are planner hints.
- These goals describe rover behavior and execution constraints, not CAD-style geometry or trajectory inputs.
- `timeout_s > 0.0` enables a deadline of that many seconds.
- `timeout_s <= 0.0` means no deadline / no timeout enforcement.
- Estimate fields use `0.0` to mean unknown or not measured.

### Result Reason Codes

- `REASON_TIMEOUT`: the goal exceeded its deadline.
- `REASON_CANCELED`: the client or higher-level controller canceled the goal.
- `REASON_SHUTDOWN`: the action server or node stopped before normal completion.

## Excavation Contract

Issue [#137](https://github.com/KJdotIO/innex1-rover/issues/137) freezes the first excavation
hardware contract.

The intended split is:

- ROS owns the excavation state machine, action handling, timing, and jam interpretation.
- Embedded owns direct motor-driver execution, limit-switch reading, and immediate hard stops.
- The interface between them is one command stream and one telemetry stream.

### Hardware-Facing Topics

- `/excavation/command` uses `lunabot_interfaces/msg/ExcavationCommand`
- `/excavation/telemetry` uses `lunabot_interfaces/msg/ExcavationTelemetry`
- `/excavation/status` uses `lunabot_interfaces/msg/ExcavationStatus`
- `/excavation/jog_forward` uses `lunabot_interfaces/srv/ExcavationJog`

### Excavation Commands

The first contract is intentionally narrow:

- `COMMAND_STOP`
- `COMMAND_HOME`
- `COMMAND_START`
- `COMMAND_CLEAR_FAULT`

`COMMAND_START` means: run the ladder forward at the embedded-side default excavation speed
after embedded-side interlock checks pass.

Anything more complicated than that belongs in a later issue once the base mechanism path is
real and tested.

### Bench Jog Contract

The first bench-tooling pass adds one controller-owned forward jog service:

- `/excavation/jog_forward` with `lunabot_interfaces/srv/ExcavationJog`

This is intentionally narrow:

- forward only
- bounded by duration
- only accepted when the controller is already `READY`
- still subject to normal stop and fault handling

The bench path must not become a raw motor bypass.

### Command Channel Rules

- `COMMAND_START` is only valid when the controller has already established a `READY` state.
- `COMMAND_CLEAR_FAULT` is only valid while faulted and does not imply a restart.
- Duplicate commands must be treated as idempotent by the excavation controller.
- `COMMAND_STOP` must always be safe to accept.
- The first implementation should use reliable QoS for both command and telemetry.

### Excavation Telemetry

The minimum excavation telemetry is:

- `estop_active`
- `driver_fault`
- `home_switch`
- `motor_enabled`
- `motor_current_a`
- `fault_code`

This is enough for a deterministic first excavation controller with explicit success, timeout,
stop, and fault handling.

The first fault-code table is:

- `FAULT_NONE=0`
- `FAULT_ESTOP=1`
- `FAULT_DRIVER=2`
- `FAULT_OVERCURRENT=3`
- `FAULT_HOME_SWITCH_INVALID=4`
- `FAULT_COMMAND_REJECTED=5`

### Excavation Status

The controller publishes a typed status snapshot for action adapters and bench tooling.

The first status-state table is:

- `STATE_IDLE=0`
- `STATE_HOMING=1`
- `STATE_READY=2`
- `STATE_STARTING=3`
- `STATE_EXCAVATING=4`
- `STATE_STOPPING=5`
- `STATE_FAULT=6`

The status topic carries the same latched fault-code table as excavation telemetry:

- `FAULT_NONE=0`
- `FAULT_ESTOP=1`
- `FAULT_DRIVER=2`
- `FAULT_OVERCURRENT=3`
- `FAULT_HOME_SWITCH_INVALID=4`
- `FAULT_COMMAND_REJECTED=5`

This topic is the controller-owned view of state. It is the supported coordination contract for
the excavation action adapter, replacing free-form string state names.

The first status fields are:

- `state`
- `fault_code`
- `estop_active`
- `driver_fault`
- `homed`
- `motor_enabled`
- `motor_current_a`

### Out of Scope for the First Contract

The following stay out of the first excavation hardware contract unless hardware sign-off makes
them real and necessary:

- fill or collected-mass estimation
- autonomous jam recovery or reverse pulses
- posture or lift-axis control
- mission-level field logic
- adaptive digging strategies

If a posture or lift axis exists, treat it as a separate actuator contract instead of hiding it
inside the ladder-motor contract.

## Drivetrain Contract

Added in PR [#248](https://github.com/KJdotIO/innex1-rover/pull/248).

### Hardware-Facing Topics

- `/drivetrain/command` uses `lunabot_interfaces/msg/DrivetrainCommand`
- `/drivetrain/status` uses `lunabot_interfaces/msg/DrivetrainStatus`
- `/drivetrain/telemetry` uses `lunabot_interfaces/msg/DrivetrainTelemetry`

### DrivetrainCommand

Per-wheel throttle command sent by the bridge to motor controllers.

- `COMMAND_DRIVE=0` — drive at the given `wheel_throttle[4]` values (FL, FR, RL, RR), range [-1.0, 1.0].
- `COMMAND_STOP=1` — immediate stop (throttle values ignored).
- `COMMAND_CLEAR_FAULT=2` — clear a latched fault (throttle values ignored).

### DrivetrainStatus

Controller-facing status published by the drivetrain bridge. Consumed by the
mission coordinator, velocity gate, and preflight checks.

State table:

- `STATE_UNINITIALISED=0`
- `STATE_READY=1`
- `STATE_DRIVING=2`
- `STATE_STOPPING=3`
- `STATE_FAULT=4`
- `STATE_ESTOP=5`

Fault codes:

- `FAULT_NONE=0`
- `FAULT_ESTOP=1`
- `FAULT_ENCODER_STALL=2`
- `FAULT_CONTROLLER_OFFLINE=3`
- `FAULT_OVERCURRENT=4`
- `FAULT_COMMAND_TIMEOUT=5`

Additional fields: `estop_active`, `motion_inhibited`, `controller_online[2]`.

### DrivetrainTelemetry

Published at the control loop rate. Per-wheel encoder feedback ordered
[FL, FR, RL, RR]:

- `wheel_velocity_rps` — angular velocity in revolutions/sec.
- `encoder_ticks` — raw cumulative tick count.
- `controller_online[2]` — true if UART heartbeat received per Sabertooth.
- `estop_active`, `motion_inhibited`, `fault_code` — mirrored safety state.

## Minimum Hardware Signals for Bridge Nodes

- `estop_active` (latched)
- per-channel motor/actuator `driver_fault`
- excavation motor `current_a`
- deposition actuator `current_a`
- excavation `home_switch`
- deposition `open` limit switch
- deposition `closed` limit switch
- deposition `raised` limit switch (if separate axis)

These signals are enough for deterministic success, timeout, and fail behaviour before final
mechanical sign-off.
