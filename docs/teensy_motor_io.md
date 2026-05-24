# Teensy Motor IO Path

This page describes the software boundary for the Teensy 4.1 motor IO work.
It is the current plan from the updated electrical CDR: the Jetson stays in ROS
2, while the Teensy owns low-level motor IO, encoder counting, and immediate
hardware interlocks.

## Runtime Shape

```text
/cmd_vel_safe
  -> velocity_gate
  -> /cmd_vel_gated
  -> teensy_drivetrain_bridge
  -> USB CDC serial
  -> Teensy 4.1
  -> Sabertooth 2x32, encoders, and later mechanism IO
```

The old direct Sabertooth path is still available for bench work:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py \
  bridge_backend:=sabertooth \
  serial_port:=/dev/ttyTHS1 \
  max_throttle:=0.2
```

The Teensy path is selected explicitly:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py \
  bridge_backend:=teensy \
  serial_port:=/dev/teensy_motor_io \
  max_throttle:=0.2
```

The Teensy backend defaults to `teensy_baud_rate:=115200`. The direct
Sabertooth backend keeps `baud_rate:=9600`.

Prefer a udev symlink such as `/dev/teensy_motor_io` on the Jetson rather than
hard-coding `/dev/ttyACM0`, because USB enumeration can move after reconnects.

## Protocol

The Jetson and Teensy use a versioned binary protocol over USB CDC serial:

- COBS framing with a zero delimiter.
- CRC-16/CCITT-FALSE over the decoded payload.
- Protocol version byte at the start of every frame.
- Fixed message IDs for heartbeat, drive command, control flags, reset faults,
  status, drivetrain telemetry, and fault events.

The host-side codec lives in:

- `src/lunabot_drivetrain/lunabot_drivetrain/teensy_protocol.py`
- `src/lunabot_drivetrain/lunabot_drivetrain/teensy_serial.py`

The firmware-side codec and safety core live in:

- `firmware/teensy_motor_io/include/teensy_motor_io/`
- `firmware/teensy_motor_io/src/`

## Sabertooth Mode

For the Teensy-to-Sabertooth link, use modern Packet Serial with CRC for the
fielded firmware. Plain Text Serial is useful for bench debugging, but it should
not be the competition control path. Legacy Simplified Serial remains available
only through the old direct Jetson bench bridge.

Set Sabertooth ramping and per-channel current limits in DEScribe before the
first powered motion test. Treat runtime packet commands for `T1`/`T2` current
targets as later polish, not the first safety line. The software must publish a
clear fault or preflight failure until electrical confirms the configured
current limit.

Use the Sabertooth serial timeout as a backstop. The Teensy host-command
watchdog should stop outputs first; the Sabertooth timeout should catch a broken
wire or dead Teensy.

## Safety Invariants

The Teensy firmware must fail closed:

- malformed COBS frames or bad CRCs are ignored;
- stale Jetson heartbeat disables outputs;
- stale `/cmd_vel_gated` disables outputs;
- E-stop and `/safety/motion_inhibit` disable outputs;
- Sabertooth current-limit/ramping configuration must be confirmed before
  drivetrain motion is allowed;
- driver faults, overcurrent, BLD-510B ALM, or invalid command ranges disable
  the affected output and publish a fault;
- clearing the physical E-stop must not resume motion by itself.

The ROS bridge also preserves the existing safety contract. Incoming Teensy
telemetry can assert E-stop or inhibit, but it must not clear the ROS E-stop or
motion-inhibit state. The combined state is what appears on
`/drivetrain/status` and `/drivetrain/telemetry`.

## Testing Without Hardware

Run the firmware core tests:

```bash
cmake -S firmware/teensy_motor_io -B build/teensy_motor_io
cmake --build build/teensy_motor_io
ctest --test-dir build/teensy_motor_io --output-on-failure
```

Run the Python protocol tests locally:

```bash
PYTHONPATH=src/lunabot_drivetrain \
  uv run --with pytest python -m pytest \
  src/lunabot_drivetrain/test/test_teensy_protocol.py \
  src/lunabot_drivetrain/test/test_teensy_serial.py -q
```

The fake Teensy endpoint prints a pseudo-terminal path that can be passed to the
bridge:

```bash
PYTHONPATH=src/lunabot_drivetrain \
  python3 -m lunabot_drivetrain.fake_teensy
```

Then launch the bridge against that PTY in another terminal.

## Open Electrical Items

- Confirm the exact Sabertooth DEScribe profile and export it with the project
  docs: Packet Serial with CRC, serial timeout, ramping, and `<=10 A`
  per-channel soft current limit.
- Finalise and label the Teensy pin map once the harness is built.
- Resolve the UK Lunabotics 2026 v1.0 E-stop wording with electrical and/or the
  organisers. Current repo notes say compute remains live; the rulebook wording
  says E-stop isolates batteries from active subsystems.
