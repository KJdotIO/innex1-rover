# Jetson to Sabertooth Bring-up

Use this when the Sabertooth motor controllers are wired to the Jetson and you
want a careful first-motion test.

> Historical bench path: this proved one-motor first motion on 2026-05-26. The
> current drivetrain wiring uses the Teensy as the low-level controller:
> `Jetson USB -> Teensy -> Sabertooth #1/#2 -> motors`, with encoders read by
> the Teensy. For the current four-motor wiring and test results, use
> `docs/teensy_drivetrain_bringup.md`.

## Wiring check

Before applying motor power, check:

- Jetson UART TX is connected to each Sabertooth `S1` serial input.
- Jetson ground and Sabertooth `0V/GND` share a common signal ground.
- Sabertooth `5V OUT` is not powering the Jetson. Use the compute power rail.
- The rover is lifted, restrained, or otherwise unable to drive away.
- The main motor-power E-stop is reachable.

The electrical CDR keeps motor power and compute power separate. That is the
right architecture: the E-stop should cut motive power while the Jetson stays
alive for logs, telemetry, and recovery.

## Jetson setup

Install the serial dependency and allow the logged-in user to open the UART:

```bash
sudo apt install python3-serial
sudo usermod -aG dialout $USER
```

Log out and back in after changing groups.

Build and source the workspace:

```bash
cd ~/innex1-rover
colcon build --symlink-install --packages-select lunabot_interfaces lunabot_teleop lunabot_bringup lunabot_drivetrain
source /opt/ros/humble/setup.bash
source install/setup.bash
```

If the Jetson workspace lives somewhere else, use that path instead of
`~/innex1-rover`.

## Validated one-motor bench setup

This setup was tested on 2026-05-26 with one drivetrain motor on Sabertooth
channel M1 and an Xbox Series X controller plugged into the Jetson.

Sabertooth 2x32 DIP switches:

```text
1 OFF
2 OFF
3 ON
4 ON
5 ON
6 ON
```

Use this with packet serial, address `128`, and `9600` baud. With a bench PSU,
DIP 3 ON is acceptable for this test. Revisit battery protection before using
the real LiPo pack.

Logic wiring:

```text
Jetson J12 pin 8 UART TX  -> Sabertooth S1
Jetson J12 pin 6 GND      -> Sabertooth 0V
```

Power and motor wiring:

```text
Bench PSU +24 V           -> Sabertooth B+
Bench PSU 0 V / negative  -> Sabertooth B-
Motor red                 -> Sabertooth M1A
Motor black               -> Sabertooth M1B
```

Do not connect either motor wire to Jetson ground, Sabertooth `0V`, or PSU
negative. `M1A` and `M1B` are both driven motor outputs, not ground terminals.

The Arduino is not needed for this bench test. Keep it disconnected from
Sabertooth `S1`; only one device should drive that serial input.

If `/dev/ttyTHS1` times out with `Failed to enable FIFO mode: -110`, reboot the
Jetson and retry before running the drivetrain bridge. The UART recovered after
a reboot during the validated test.

To prove the serial path before using teleop, send one short packetised command:

```bash
python3 -c 'import time, serial; p=lambda a,c,v: bytes([a,c,v,(a+c+v)&0x7F]); s=serial.Serial("/dev/ttyTHS1",9600,timeout=0.01); s.write(p(128,0,100)); s.flush(); time.sleep(2.0); s.write(p(128,0,0)+p(128,4,0)); s.flush(); s.close()'
```

This commands M1 forward at packet value `100` for two seconds, then stops M1
and M2.

## Start the drivetrain bridge

Start with a low throttle cap:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py max_throttle:=0.2
```

This launch starts the drivetrain bridge and velocity gate. Keep
`dry_run:=false` for real first-motion tests: if the serial controller is
missing, the bridge faults and the gate stays closed. Use `dry_run:=true` only
when you deliberately want a no-motor-output dry run.

If the Sabertooth UART is not `/dev/ttyTHS1`, pass the actual device:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py serial_port:=/dev/ttyTHS0 max_throttle:=0.2
```

The CDR calls the Sabertooth serial wiring "serial". For the validated bench
setup above, pass `serial_protocol:=packetized baud_rate:=9600`. Use the launch
default only when the Sabertooth DIP switches and wiring have deliberately been
set for Legacy Simplified Serial.

## Send a low-speed motion command

In a second terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/innex1-rover/install/setup.bash
ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

The bridge automatically stops the motors if it does not receive another command
within `command_timeout_s`.

Send an explicit stop:

```bash
ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Watch the drivetrain state:

```bash
ros2 topic echo /drivetrain/status
ros2 topic echo /drivetrain/telemetry
```

## Use a controller

Plug in the controller, then launch teleop through the same bench file:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py enable_teleop:=true max_throttle:=0.2 serial_protocol:=packetized baud_rate:=9600
```

If the controller is not device `0`, try:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py enable_teleop:=true joy_device_id:=1 max_throttle:=0.2 serial_protocol:=packetized baud_rate:=9600
```

Hold Xbox button `9` while moving the left stick. Start with
`max_throttle:=0.2`; `max_throttle:=0.7` was also tested successfully on the
single-motor bench setup once the low-speed test passed.

The signal path is:

```text
game_controller_node
  -> /joy
  -> teleop_twist_joy
  -> /cmd_vel_teleop
  -> twist_mux
  -> /cmd_vel_safe
  -> velocity_gate
  -> /cmd_vel_gated
  -> drivetrain_bridge
  -> Jetson UART TX
  -> Sabertooth S1
```

## micro-ROS?

Not for first motion.

micro-ROS is useful if an Arduino is staying in the robot as a ROS-aware
microcontroller for encoder counting, watchdogs, hard real-time IO, or extra
sensors. For this drivetrain bring-up, the shortest reliable chain is:

```text
Jetson ROS 2 node -> USB serial -> Teensy -> Sabertooth motor controllers
```

The Teensy is not running micro-ROS in the current bench firmware. It exposes a
small line-based USB serial protocol and owns encoder counting, ramping,
watchdog timeout, and latched stop/restart behaviour.
