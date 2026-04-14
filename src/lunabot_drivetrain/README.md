# lunabot_drivetrain

Drivetrain bridge for the INNEX-1 four-wheel skid-steer rover. Converts
`cmd_vel` to Sabertooth Packetized Serial commands over UART and publishes
encoder-derived odometry and telemetry.

## Hardware

- **Motors**: 4x GR-WM4-V4 brushed DC with Hall-sensor quadrature encoders.
- **Controllers**: 2x Sabertooth 2x32 on a single UART (`/dev/ttyTHS1`,
  9600 baud). DIP-switch addresses 128 (FL+FR) and 129 (RL+RR).
- **Protocol**: Packetized Serial — 4-byte frames
  `[address, command, data, checksum]`.

If the serial port is unavailable at startup the bridge enters **dry-run
mode**: no motor output, but all ROS interfaces remain active. This allows
desktop testing without hardware.

## Nodes

### `drivetrain_bridge`

Differential-drive kinematics: splits `Twist` into per-side throttles, sends
them to both Sabertooth controllers, and publishes status/telemetry.

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `/cmd_vel_safe` | `geometry_msgs/Twist` |
| Subscribe | `/safety/motion_inhibit` | `std_msgs/Bool` (transient-local) |
| Subscribe | `/safety/estop` | `std_msgs/Bool` |
| Publish | `/drivetrain/status` | `lunabot_interfaces/DrivetrainStatus` |
| Publish | `/drivetrain/telemetry` | `lunabot_interfaces/DrivetrainTelemetry` |
| Publish | `/odom_wheels` | `nav_msgs/Odometry` |
| Publish | `/joint_states_wheels` | `sensor_msgs/JointState` |

State machine: `UNINITIALISED → READY ⇄ DRIVING → FAULT` and `ESTOP`.

- **Stall detection**: if throttle > threshold but encoder velocity ≈ 0 for
  `stall_timeout_s`, the bridge faults with `FAULT_ENCODER_STALL`.
- **E-stop recovery**: after `/safety/estop` clears, the bridge waits 2 s for
  the Sabertooth to reboot before transitioning back to READY.
- **Command timeout**: if no `cmd_vel` arrives within `command_timeout_s`, motors
  are stopped.

### `velocity_gate`

Safety filter between the navigation stack and the drivetrain. Forwards
`/cmd_vel_safe` to `/cmd_vel_gated` only when the drivetrain is in a healthy
state (READY or DRIVING, no E-stop, no motion inhibit). Otherwise publishes
zero twist.

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `/cmd_vel_safe` | `geometry_msgs/Twist` |
| Subscribe | `/drivetrain/status` | `lunabot_interfaces/DrivetrainStatus` |
| Subscribe | `/safety/motion_inhibit` | `std_msgs/Bool` (transient-local) |
| Publish | `/cmd_vel_gated` | `geometry_msgs/Twist` |

## Key files

- `config/drivetrain.yaml`: all tuneable parameters (kinematics, encoder CPR,
  stall thresholds, control/telemetry rates).
- `lunabot_drivetrain/sabertooth_serial.py`: low-level Packetized Serial
  framing and `send_throttle`/`send_stop` helpers.

## Common failure modes

- Serial port permission denied on Jetson (`sudo usermod -aG dialout $USER`).
- Wrong Sabertooth DIP-switch address — bridge writes succeed but motors don't
  respond.
- Stall false-positives during bringup if encoder GPIO pins are not wired
  (reduce `max_throttle` or raise `stall_throttle_threshold` for bench tests).
