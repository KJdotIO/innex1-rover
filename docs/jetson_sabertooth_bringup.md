# Jetson to Sabertooth Bring-up

Use this when the Sabertooth motor controllers are wired to the Jetson and you
want a careful first-motion test.

## Wiring check

Before applying motor power, check:

- Jetson UART TX is connected to each Sabertooth `S1` serial input.
- Jetson ground and Sabertooth `0V/GND` share a common signal ground.
- Sabertooth DIP switches are set for Legacy Simplified Serial at `9600` baud.
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

The CDR calls this Simplified Serial. In the Sabertooth 2x32 manual, the
byte-based Arduino-style mode is Legacy Simplified Serial, so the launch file
uses `serial_protocol:=legacy_simplified` by default. Use
`serial_protocol:=packetized` only if the Sabertooth DIP switches are set for
Packet Serial instead.

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
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py enable_teleop:=true max_throttle:=0.2
```

If the controller is not device `0`, try:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py enable_teleop:=true joy_device_id:=1 max_throttle:=0.2
```

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
Jetson ROS 2 node -> UART -> Sabertooth motor controller
```

Keep the Arduino out of the path until there is a specific job that the Jetson
should not be doing directly.
