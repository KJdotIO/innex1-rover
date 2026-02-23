# Material Handling Interface Notes

This package contains mechanism-agnostic action contracts for excavation and deposition.

## Actions

- `/mission/excavate` uses `lunabot_interfaces/action/Excavate`
- `/mission/deposit` uses `lunabot_interfaces/action/Deposit`

## Minimum Hardware Signals for Bridge Nodes

- `estop_active` (latched)
- per-channel motor/actuator `driver_fault`
- excavation motor `current_a`
- deposition actuator `current_a`
- excavation `home/retracted` limit switch
- deposition `open` limit switch
- deposition `closed` limit switch
- deposition `raised` limit switch (if separate axis)

These signals are enough for deterministic success, timeout, and fail behaviour before final
mechanical sign-off.
