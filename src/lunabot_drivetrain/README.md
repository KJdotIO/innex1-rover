# lunabot_drivetrain

Drivetrain bridge for the INNEX-1 four-wheel skid-steer rover. Converts
`cmd_vel` to Sabertooth serial commands over UART and publishes
encoder-derived odometry and telemetry.

## Hardware

- **Motors**: 4x GR-WM4-V4 brushed DC with Hall-sensor quadrature encoders.
- **Controllers**: 2x Sabertooth 2x32 on Jetson UART (`/dev/ttyTHS1`,
  9600 baud).
- **Default protocol**: Legacy Simplified Serial, matching the electrical CDR's
  "Simplified Serial" wiring. Packet Serial is also supported by setting
  `serial_protocol:=packetized`.

Real hardware launches fail closed if the serial port is unavailable. For
desktop or Jetson dry-runs without motor output, set `dry_run:=true`
explicitly.

## Nodes

### `drivetrain_bridge`

Differential-drive kinematics: splits `Twist` into per-side throttles, sends
them to both Sabertooth controllers, and publishes status/telemetry.

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `/cmd_vel_gated` | `geometry_msgs/Twist` |
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

## Jetson bench bring-up

Use this path when the Sabertooths are wired to the Jetson and you want a small,
controlled first-motion test.

For the electrician-friendly checklist, see
`docs/jetson_sabertooth_bringup.md` from the repository root.

Before powering motor voltage, confirm:

- Sabertooth `0V/GND` is tied to Jetson ground.
- Jetson UART TX goes to each Sabertooth serial input (`S1`) as shown in the
  electrical CDR.
- The Sabertooth DIP switches are set for Legacy Simplified Serial at `9600`
  baud, or `serial_protocol` and `baud_rate` are changed to match the hardware.
- The rover is lifted or restrained for the first test, with motor power ready
  to isolate immediately.

On the Jetson, install the serial dependency and give the user access to the
UART:

```bash
sudo apt install python3-serial
sudo usermod -aG dialout $USER
```

Log out and back in after changing groups. If `/dev/ttyTHS1` is not the wired
UART, pass the actual device as `serial_port:=...`.

Build and source the workspace:

```bash
cd ~/innex1-rover
colcon build --symlink-install --packages-select lunabot_interfaces lunabot_teleop lunabot_bringup lunabot_drivetrain
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Start the bridge with a low throttle cap:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py max_throttle:=0.2
```

In a second terminal, send a short forward command:

```bash
source /opt/ros/humble/setup.bash
source ~/innex1-rover/install/setup.bash
ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

The bridge will stop the motors automatically after `command_timeout_s` if no
new command arrives. To send an explicit stop:

```bash
ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Watch status while testing:

```bash
ros2 topic echo /drivetrain/status
ros2 topic echo /drivetrain/telemetry
```

For controller testing, plug in the gamepad and start the same launch file with
teleop enabled:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py enable_teleop:=true max_throttle:=0.2
```

The joystick path is:

`game_controller_node` → `/joy` → `teleop_twist_joy` → `/cmd_vel_teleop` →
`twist_mux` → `/cmd_vel_safe` → `velocity_gate` → `/cmd_vel_gated` →
`drivetrain_bridge` → Sabertooth serial.

If the controller is not picked up as device `0`, try:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py enable_teleop:=true joy_device_id:=1
```

or match it by SDL name:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py enable_teleop:=true joy_device_name:="Xbox Wireless Controller"
```

## micro-ROS?

micro-ROS is useful when the Arduino is staying in the system as a ROS-aware
microcontroller. For this drivetrain, it is not the simplest first step: the
Jetson can already publish ROS 2 velocity commands and this package already
speaks Sabertooth serial.

Use micro-ROS later if the Arduino needs to remain responsible for hard
real-time IO, encoder counting, watchdogs, or extra sensors. For first motion,
keep the chain shorter: Jetson ROS 2 node → UART → Sabertooth.

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
- `lunabot_drivetrain/sabertooth_serial.py`: low-level Simplified Serial and
  Packet Serial framing helpers.

## Common failure modes

- Serial port permission denied on Jetson (`sudo usermod -aG dialout $USER`).
- Wrong Sabertooth DIP-switch protocol or baud rate: bridge writes succeed but
  motors don't respond.
- Stall false-positives during bringup if encoder GPIO pins are not wired
  (reduce `max_throttle` or raise `stall_throttle_threshold` for bench tests).
