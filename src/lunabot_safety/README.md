# lunabot_safety

Scaffolding package for centralised safety logic on the INNEX-1 rover.

## Current state

This package contains a single node. The bulk of the rover's safety behaviour
is currently distributed across other packages:

| Safety function | Where it lives |
|-----------------|----------------|
| E-stop → motion inhibit bridge | `lunabot_safety` (`estop_node`) |
| Velocity gating (zero cmd_vel on fault) | `lunabot_drivetrain` (`velocity_gate`) |
| Drivetrain stall detection & E-stop recovery | `lunabot_drivetrain` (`drivetrain_bridge`) |
| Deposition safety callbacks (E-stop + inhibit) | `lunabot_control` (`deposition_bridge`) |
| Mission-level safety gating | `lunabot_bringup` (`mission_manager`) |

## Nodes

### `estop_node`

Bridges the hardware E-stop signal to a latched motion-inhibit topic.

| Direction | Topic | Type | QoS |
|-----------|-------|------|-----|
| Subscribe | `/safety/estop` | `std_msgs/Bool` | default (reliable, volatile) |
| Publish | `/safety/motion_inhibit` | `std_msgs/Bool` | reliable, transient-local |

Transient-local durability ensures that nodes joining after an E-stop event
(e.g. a motor controller that restarts) immediately receive the current
inhibit state rather than waiting for the next event.

## Safety topic convention

All safety-aware nodes subscribe to `/safety/estop` and/or
`/safety/motion_inhibit`. The inhibit topic uses `TRANSIENT_LOCAL` QoS
throughout the stack so late joiners get the latest value.
