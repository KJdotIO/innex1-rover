# Teensy Bench Tools

These tools were used during the 2026-05-27 drivetrain bench session.

The validated hardware path is:

```text
Mac or Jetson USB serial -> Teensy 4.1
Teensy pin 1 -> left Sabertooth S1 -> FL/RL motors
Teensy pin 8 -> right Sabertooth S1 -> FR/RR motors
Teensy pins 15-22 <- FL/RL/FR/RR quadrature encoders
```

See `docs/teensy_drivetrain_bringup.md` for the wiring tables, safety notes,
captured encoder counts, Jetson integration notes, and ROS bridge results.

## Sketches

| Path | Purpose |
|------|---------|
| `encoder_smoke/encoder_smoke.ino` | Single-encoder smoke test. |
| `sabertooth_encoder_bench/sabertooth_encoder_bench.ino` | One-motor Sabertooth plus encoder bench test. |
| `drivetrain_serial_firmware/drivetrain_serial_firmware.ino` | Current four-motor Teensy firmware. |

## Current Firmware Protocol

USB serial baud: `115200`.

```text
V <left> <right>   # side throttle targets, -127..127
X                  # hard stop
E                  # E-stop active: hard stop + latch inhibit
U                  # E-stop released, restart still required
R                  # restart/reset inhibit
Z                  # reset encoders
H                  # help
```

Telemetry is emitted at 10 Hz:

```text
T <ms> <state> <estop> <inhibit> <targetL> <targetR> <cmdL> <cmdR> <FL> <RL> <FR> <RR>
```

State values:

| State | Meaning |
|-------|---------|
| `0` | Ready |
| `1` | Driving |
| `2` | Inhibited; restart required |
| `3` | E-stop active |
| `4` | Command timeout |

## Upload

On the lab Mac, the Teensy 4.1 appeared as `/dev/cu.usbmodem198521001` during
testing. The exact device path can change after reconnecting.

```bash
env -u CPLUS_INCLUDE_PATH -u SDKROOT arduino-cli compile \
  --fqbn teensy:avr:teensy41 \
  tools/teensy/drivetrain_serial_firmware

env -u CPLUS_INCLUDE_PATH -u SDKROOT arduino-cli upload \
  -p usb:1110000 \
  --fqbn teensy:avr:teensy41 \
  tools/teensy/drivetrain_serial_firmware
```

## Smoke Tests

Use `teensy_drive_smoke.py` from the repository root. It uses only the Python
standard library; no `pyserial` install is needed.

```bash
# Smooth all-forward test
python3 tools/teensy/teensy_drive_smoke.py --left 60 --right 60 --duration 3

# Side isolation
python3 tools/teensy/teensy_drive_smoke.py --left 35 --right 0 --duration 2.5
python3 tools/teensy/teensy_drive_smoke.py --left 0 --right 35 --duration 2.5

# Skid-steer turns
python3 tools/teensy/teensy_drive_smoke.py --left 30 --right 60 --duration 3
python3 tools/teensy/teensy_drive_smoke.py --left 60 --right 30 --duration 3
python3 tools/teensy/teensy_drive_smoke.py --left 40 --right -40 --duration 2.5

# Latched stop behaviour
python3 tools/teensy/teensy_drive_smoke.py --left 80 --right 0 --duration 5 --estop-after 2
```

Known-good final bench results are recorded in
`docs/teensy_drivetrain_bringup.md`.
