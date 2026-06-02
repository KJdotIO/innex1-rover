# lunabot_teleop

Teleoperation package for the INNEX-1 rover.

The current competition-shaped path avoids running ROS 2 on the operator laptop.
The Jetson hosts a browser Gamepad page, the laptop sends small HTTPS commands
over the rover Wi-Fi, and the Jetson publishes `/cmd_vel_safe`.

```text
browser Gamepad API
  -> Jetson web_gamepad_bridge
  -> /cmd_vel_safe
  -> velocity_gate
  -> /cmd_vel_gated
  -> drivetrain_bridge
  -> Teensy USB serial
  -> Sabertooth controllers
```

Do not bypass the velocity gate for normal driving.

## Joystick Teleop

Use this when the controller is plugged directly into the Jetson:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py \
  enable_teleop:=true \
  max_throttle:=0.2
```

The joystick launch publishes `/cmd_vel_teleop`. The drivetrain bench launch
starts `twist_mux`, which forwards that into `/cmd_vel_safe`.

If the controller is not device `0`, pass a device ID:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py \
  enable_teleop:=true \
  joy_device_id:=1 \
  max_throttle:=0.2
```

Or match by SDL name:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py \
  enable_teleop:=true \
  joy_device_name:="Xbox Wireless Controller" \
  max_throttle:=0.2
```

## Browser Gamepad Bridge

Use this when the controller is attached to the operator laptop.

Generate a short-lived lab certificate on the Jetson:

```bash
openssl req -x509 -newkey rsa:2048 -nodes \
  -keyout /tmp/lunabot_web_gamepad.key \
  -out /tmp/lunabot_web_gamepad.crt \
  -days 7 \
  -subj "/CN=innex1-web-gamepad" \
  -addext "subjectAltName=IP:192.168.8.20,DNS:innex1-desktop"
```

Start the bridge:

```bash
ros2 launch lunabot_teleop web_gamepad_bridge.launch.py \
  bind_host:=0.0.0.0 \
  port:=9443 \
  tls_cert_file:=/tmp/lunabot_web_gamepad.crt \
  tls_key_file:=/tmp/lunabot_web_gamepad.key
```

Then open this from the operator laptop while it is connected to the rover
router:

```text
https://192.168.8.20:9443
```

The browser will warn about the self-signed certificate. Accept it only for the
rover address.

The bridge refuses to bind to all interfaces without TLS. It also embeds a
per-run operator token in the served page, and POST requests must include that
token.

## State API

The bridge exposes a small status endpoint:

```text
GET /api/state
```

It reports:

- current speed limits;
- command freshness;
- drivetrain status;
- drivetrain telemetry.

## Stop Behaviour

The bridge publishes zero when:

- the browser command stream goes stale;
- the enable control is released;
- the page closes or Wi-Fi drops for longer than `command_timeout_s`;
- the node shuts down.

The drivetrain still has its own command timeout and the Teensy firmware still
owns its local watchdog, so a stale browser is not the only stop path.

## Useful Checks

```bash
ss -ltnp | egrep ':(9443)'
curl -k https://192.168.8.20:9443/api/state
ros2 topic echo /cmd_vel_safe
ros2 topic echo /cmd_vel_gated
ros2 topic echo /drivetrain/status
```

If `/cmd_vel_safe` changes but `/cmd_vel_gated` stays zero, check
`/safety/estop`, `/safety/motion_inhibit`, and `/drivetrain/status` before
changing controller settings.
