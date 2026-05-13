# lunabot_safety

Safety package for the E-stop to motion-inhibit bridge on the INNEX-1 rover.

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

Latches the hardware E-stop signal into a motion-inhibit topic. When
`/safety/estop` becomes true, `/safety/motion_inhibit` is set true and stays
true after the E-stop input clears. Motion is only allowed again after an
operator publishes a true message on `/safety/reset_motion_inhibit`.

| Direction | Topic | Type | QoS |
|-----------|-------|------|-----|
| Subscribe | `/safety/estop` | `std_msgs/Bool` | default (reliable, volatile) |
| Subscribe | `/safety/reset_motion_inhibit` | `std_msgs/Bool` | default (reliable, volatile) |
| Publish | `/safety/motion_inhibit` | `std_msgs/Bool` | reliable, transient-local |

Transient-local durability ensures that nodes joining after an E-stop event
(e.g. a motor controller that restarts) immediately receive the current
inhibit state rather than waiting for the next event.

The reset topic is deliberately separate from `/safety/estop`. Releasing a
physical E-stop should not make the rover move again by itself.

```bash
ros2 topic pub --once /safety/reset_motion_inhibit std_msgs/msg/Bool "{data: true}"
```

The node ignores reset requests while `/safety/estop` is still true.

## Safety topic convention

All safety-aware nodes subscribe to `/safety/estop` and/or
`/safety/motion_inhibit`. The inhibit topic uses `TRANSIENT_LOCAL` QoS
throughout the stack so late joiners get the latest value.
