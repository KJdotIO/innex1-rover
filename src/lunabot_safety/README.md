# lunabot_safety

Safety package for the E-stop to motion-inhibit bridge on the INNEX-1 rover.

## Current state

Two nodes live here. The rest of the rover's safety behaviour is currently
distributed across other packages:

| Safety function | Where it lives |
|-----------------|----------------|
| Physical E-stop input publisher | `lunabot_safety` (`physical_estop_input`) |
| E-stop → motion inhibit bridge | `lunabot_safety` (`estop_node`) |
| Velocity gating (zero cmd_vel on fault) | `lunabot_drivetrain` (`velocity_gate`) |
| Drivetrain stall detection & E-stop recovery | `lunabot_drivetrain` (`drivetrain_bridge`) |
| Deposition safety callbacks (E-stop + inhibit) | `lunabot_control` (`deposition_bridge`) |
| Mission-level safety gating | `lunabot_bringup` (`mission_manager`) |

## Nodes

Start both safety nodes with:

```bash
ros2 launch lunabot_safety safety.launch.py
```

For a confirmed Jetson GPIO E-stop input:

```bash
ros2 launch lunabot_safety safety.launch.py \
  estop_backend:=jetson_gpio \
  estop_gpio_pin:=<pin> \
  estop_active_high:=true
```

### `physical_estop_input`

Publishes the physical E-stop state on `/safety/estop`. The default
`backend:=parameter` mode is for bench and CI checks. Hardware runs should use
`backend:=jetson_gpio` only after the actual pin, polarity, and wiring have
been confirmed.

| Direction | Topic | Type | QoS |
|-----------|-------|------|-----|
| Publish | `/safety/estop` | `std_msgs/Bool` | default (reliable, volatile) |

Useful parameters:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `backend` | `parameter` | `parameter` or `jetson_gpio` |
| `gpio_pin` | `0` | Jetson GPIO pin number for hardware mode |
| `gpio_numbering` | `BOARD` | `BOARD` or `BCM` numbering |
| `active_high` | `true` | Whether a high input means E-stop active |
| `simulated_estop_active` | `false` | Published state in parameter mode |
| `release_debounce_samples` | `3` | Consecutive clear samples before release |
| `fail_safe_on_error` | `true` | Publish active E-stop if GPIO cannot be read |

E-stop assertion is immediate. Only release is debounced.

### `estop_node`

Latches the hardware E-stop signal into a motion-inhibit topic. When
`/safety/estop` becomes true, `/safety/motion_inhibit` is set true and stays
true after the E-stop input clears. Motion is only allowed again after an
operator publishes a true message on `/safety/reset_motion_inhibit`.

Critical low-voltage telemetry from `/power/telemetry` is treated the same way:
it latches `/safety/motion_inhibit` until the voltage recovers and the operator
explicitly resets motion inhibit.

| Direction | Topic | Type | QoS |
|-----------|-------|------|-----|
| Subscribe | `/safety/estop` | `std_msgs/Bool` | default (reliable, volatile) |
| Subscribe | `/safety/reset_motion_inhibit` | `std_msgs/Bool` | default (reliable, volatile) |
| Subscribe | `/power/telemetry` | `lunabot_interfaces/msg/PowerTelemetry` | default (reliable, volatile) |
| Publish | `/safety/motion_inhibit` | `std_msgs/Bool` | reliable, transient-local |

Transient-local durability ensures that nodes joining after an E-stop event
(e.g. a motor controller that restarts) immediately receive the current
inhibit state rather than waiting for the next event.

The reset topic is deliberately separate from `/safety/estop`. Releasing a
physical E-stop should not make the rover move again by itself.

```bash
ros2 topic pub --once /safety/reset_motion_inhibit std_msgs/msg/Bool "{data: true}"
```

The node ignores reset requests while `/safety/estop` is still true or power
telemetry is still critical.

## Safety topic convention

All safety-aware nodes subscribe to `/safety/estop` and/or
`/safety/motion_inhibit`. The inhibit topic uses `TRANSIENT_LOCAL` QoS
throughout the stack so late joiners get the latest value.
