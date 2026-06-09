# lunabot_drivetrain

Drivetrain bridge for the INNEX-1 four-wheel skid-steer rover. The older bridge
converts `cmd_vel` directly to Sabertooth serial commands over UART. The current
bench-proven hardware path is now Teensy-based: Jetson USB serial to Teensy,
then Teensy packet serial to the two Sabertooth controllers.

## Hardware

- **Motors**: 4x GR-WM4-V3 brushed DC with Hall-sensor quadrature encoders.
- **Bench-proven controller path**: Jetson USB serial to Teensy 4.1; Teensy pin
  `1` to left Sabertooth `S1`; Teensy pin `8` to right Sabertooth `S1`.
- **Historical direct path**: 2x Sabertooth 2x32 on Jetson UART
  (`/dev/ttyTHS1`, 9600 baud). This proved one-motor first motion, but is not
  the final four-motor wiring.
- **Current Teensy protocol**: line-based USB serial, documented in
  `docs/teensy_drivetrain_bringup.md` and implemented in
  `tools/teensy/drivetrain_serial_firmware/`.

Real hardware launches fail closed if the serial port is unavailable. For
desktop or Jetson dry-runs without motor output, set `dry_run:=true`
explicitly.

The normal bench path is now:

`/cmd_vel_safe` → `velocity_gate` → `/cmd_vel_gated` → `drivetrain_bridge` →
Teensy USB serial → left/right Sabertooth S1 pins.

## Nodes

### `drivetrain_bridge`

Differential-drive kinematics: splits `Twist` into per-side throttles, sends
them to the configured serial backend, and publishes status/telemetry. With
`serial_protocol:=teensy_line`, the bridge also reads encoder ticks from the
Teensy `T ...` telemetry line.

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `/cmd_vel_gated` | `geometry_msgs/Twist` |
| Subscribe | `/deposition/actuator/cmd` | `std_msgs/Int8MultiArray` |
| Subscribe | `/excavation/bldc/cmd` | `std_msgs/Int8` |
| Subscribe | `/safety/motion_inhibit` | `std_msgs/Bool` (transient-local) |
| Subscribe | `/safety/estop` | `std_msgs/Bool` |
| Publish | `/excavation/bldc/feedback` | `std_msgs/Int32MultiArray` |
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
- **Actuator timeout**: Cytron actuator outputs are hold-to-run. Non-zero
  `/actuator/cmd` and `/deposition/actuator/cmd` commands must be refreshed
  before `actuator_command_timeout_s`, otherwise the bridge sends `[0, 0]` to
  the relevant Cytron so a missed stop message cannot leave an actuator latched.
- **Excavation BLDC**: with `serial_protocol:=teensy_line`,
  `/excavation/bldc/cmd` forwards signed speed commands to the Teensy firmware
  `B <speed>` command. Valid values are clamped to `-127..127`; publish `0` to
  stop the BLDC. The bridge refreshes the active non-zero BLDC command at a
  bounded rate so the firmware watchdog remains fed, and sends `0` if the ROS
  command becomes stale. `/excavation/bldc/feedback` publishes
  `[commanded_speed, pg_pulse_count, alarm_active]`, where `alarm_active` is `1`
  when the BLD-510B `ALM` line is pulled low.

## Jetson bench bring-up

Use the direct-Sabertooth path only when the Sabertooths are deliberately wired
to the Jetson for a small, controlled first-motion test. For the current
four-motor setup, use the Teensy path in `docs/teensy_drivetrain_bringup.md`.

For the electrician-friendly checklist, see
`docs/jetson_sabertooth_bringup.md` from the repository root.

Before powering motor voltage, confirm:

- Teensy GND, both Sabertooth `0V/GND` pins, and encoder grounds are common.
- Teensy pin `1` goes to the left Sabertooth `S1`.
- Teensy pin `8` goes to the right Sabertooth `S1`.
- The Teensy USB serial device is visible on the Jetson as `/dev/ttyACM0` or a
  stable `/dev/serial/by-id/...` path.
- The rover is lifted or restrained for the first test, with motor power ready
  to isolate immediately.

On the Jetson, install the serial dependency and give the user access to the
UART:

```bash
sudo apt install python3-serial
sudo usermod -aG dialout $USER
```

Log out and back in after changing groups. If `/dev/ttyACM0` is not the Teensy,
pass the actual device as `serial_port:=...`.

Build and source the workspace:

```bash
cd ~/innex1-rover
colcon build --symlink-install --packages-select lunabot_interfaces lunabot_teleop lunabot_bringup lunabot_drivetrain
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Start the bridge with the bench throttle cap:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py max_throttle:=0.8
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

For wired Jetson bench teleop, plug the gamepad into the Jetson and start the
same launch file with teleop enabled:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py enable_teleop:=true max_throttle:=0.25
```

The wired Jetson joystick path is:

`game_controller_node` → `/joy` → `teleop_twist_joy` → `/cmd_vel_teleop` →
`twist_mux` → `/cmd_vel_safe` → `velocity_gate` → `/cmd_vel_gated` →
`drivetrain_bridge` → Teensy USB serial → Sabertooth serial.

If the controller is not picked up as device `0`, try:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py enable_teleop:=true joy_device_id:=1
```

or match it by SDL name:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py enable_teleop:=true joy_device_name:="Xbox Wireless Controller"
```

For operator-laptop teleop over rover Wi-Fi, keep the drivetrain launch running
on the Jetson with `enable_teleop:=false`, then start the browser Gamepad
bridge:

```bash
ros2 launch lunabot_teleop web_gamepad_bridge.launch.py \
  bind_host:=0.0.0.0 \
  port:=9443 \
  tls_cert_file:=/tmp/lunabot_web_gamepad.crt \
  tls_key_file:=/tmp/lunabot_web_gamepad.key
```

The laptop opens `https://<jetson-ip>:9443`, reads the Xbox controller through
the browser Gamepad API, and sends bounded commands into `/cmd_vel_safe`.

## micro-ROS?

micro-ROS is useful when a microcontroller is staying in the system as a
ROS-aware endpoint. The current Teensy bench firmware is simpler: it speaks a
small USB serial protocol and owns encoder counting, ramping, watchdog timeout,
and stop/restart behaviour.

Use micro-ROS later only if the Teensy needs to become a ROS-aware node. For the
next integration step, add a normal ROS 2 serial bridge on the Jetson:
`/cmd_vel_gated -> Teensy USB serial`.

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
- `lunabot_drivetrain/teensy_serial.py`: current Teensy USB serial line
  protocol helpers.
- `lunabot_drivetrain/sabertooth_serial.py`: historical low-level Simplified
  Serial and Packet Serial framing helpers.
- `docs/teensy_drivetrain_bringup.md`: current four-motor Teensy wiring and
  bench results.
- `tools/teensy/`: current low-level Teensy firmware and serial smoke tests.

## Common failure modes

- Serial port permission denied on Jetson (`sudo usermod -aG dialout $USER`).
- Teensy opened from a raw terminal with echo enabled: boot text can be echoed
  back into the firmware and latch software e-stop. The ROS bridge uses
  pyserial and flushes startup chatter.
- Wrong Sabertooth DIP-switch protocol or baud rate downstream of the Teensy:
  bridge writes succeed but motors don't respond.
- Stall false-positives during bring-up if encoder channels are unplugged
  (reduce `max_throttle` or raise `stall_throttle_threshold` for bench tests).
