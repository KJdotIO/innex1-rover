# Material Handling Interface Notes

This package contains mechanism-agnostic action contracts for excavation and deposition.

## Actions

- `/mission/excavate` uses `lunabot_interfaces/action/Excavate`
- `/mission/deposit` uses `lunabot_interfaces/action/Deposit`

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
