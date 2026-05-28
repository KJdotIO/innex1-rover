# Teensy Drivetrain Bring-up

Use this for bench testing the drivetrain through the intended low-level path:

```text
Mac or Jetson USB serial -> Teensy 4.1 -> two Sabertooth 2x32 controllers -> four GR-WM4-V3 motors
```

The older Jetson-to-Sabertooth path proved first motion. This path is the one to
use for the rover drivetrain because the Teensy owns the watchdog, ramping,
encoder counting, and restart-after-E-stop behaviour.

## Safety Setup

Before applying motor power:

- Keep the rover lifted, restrained, or otherwise unable to drive away.
- Power the Teensy over USB before turning on the 24 V motor supply.
- Keep the bench PSU current limit conservative for first tests.
- Do not connect Sabertooth `5V` to the Teensy.
- Do not treat motor black wires as ground. `M1A/M1B` and `M2A/M2B` are both
  driven motor outputs.
- Share signal ground: Teensy `GND` must connect to Sabertooth `0V`.

Turn the PSU off before changing motor, encoder, or Sabertooth wiring.

## Sabertooth Assignment

| Controller | Side | Teensy TX | Sabertooth Signal | Motors |
|------------|------|-----------|-------------------|--------|
| Sabertooth #1 | Left | Pin `1` | `S1` | FL on `M1`, RL on `M2` |
| Sabertooth #2 | Right | Pin `8` | `S1` | FR on `M1`, RR on `M2` |

Both Sabertooths use packet serial, address `128`, at `9600` baud. Match the
right controller DIP switches to the working left controller before testing.

The two `S1` wires must not be tied together. A 2026-05-27 bench fault had both
Sabertooth `S1` inputs wired to Teensy pin `1`, which made all four motors
follow the left-side command and prevented real skid steering. The isolation
test for this is simple:

```text
V 35 0 -> only FL/RL should move
V 0 35 -> only FR/RR should move
```

## Left Side Wiring

This was bench-tested on 2026-05-27.

| Item | Wire | Connection |
|------|------|------------|
| Left Sabertooth signal | `S1` | Teensy pin `1` |
| Left Sabertooth ground | `0V` | Teensy `GND` |
| Left Sabertooth power | `B+` | PSU `+24 V` / motive positive distribution |
| Left Sabertooth power | `B-` | PSU negative / motive 0 V distribution |
| Front-left motor | Thick red | Left Sabertooth `M1A` |
| Front-left motor | Thick black | Left Sabertooth `M1B` |
| Rear-left motor | Thick red | Left Sabertooth `M2A` |
| Rear-left motor | Thick black | Left Sabertooth `M2B` |
| FL encoder VCC | Red | Teensy `3.3 V` distribution |
| FL encoder GND | Black | Teensy `GND` distribution |
| FL encoder A | White | Teensy pin `15` |
| FL encoder B | Yellow | Teensy pin `16` |
| RL encoder VCC | Red | Teensy `3.3 V` distribution |
| RL encoder GND | Black | Teensy `GND` distribution |
| RL encoder A | White | Teensy pin `17` |
| RL encoder B | Yellow | Teensy pin `18` |

Bench result with `V 80 0` for 3 seconds:

```text
FL encoder: +2191 ticks
RL encoder: +2197 ticks
```

The rear-left motor initially spun opposite to the front-left. Swapping the
rear-left motor leads at `M2A/M2B` corrected the physical direction; the encoder
phase then matched the front-left encoder.

## Right Side Wiring

Wire this as the mirror of the tested left side.

| Item | Wire | Connection |
|------|------|------------|
| Right Sabertooth signal | `S1` | Teensy pin `8` |
| Right Sabertooth ground | `0V` | Teensy `GND` |
| Right Sabertooth power | `B+` | PSU `+24 V` / motive positive distribution |
| Right Sabertooth power | `B-` | PSU negative / motive 0 V distribution |
| Front-right motor | Thick red | Right Sabertooth `M1A` |
| Front-right motor | Thick black | Right Sabertooth `M1B` |
| Rear-right motor | Thick red | Right Sabertooth `M2A` |
| Rear-right motor | Thick black | Right Sabertooth `M2B` |
| FR encoder VCC | Red | Teensy `3.3 V` distribution |
| FR encoder GND | Black | Teensy `GND` distribution |
| FR encoder A | White | Teensy pin `19` |
| FR encoder B | Yellow | Teensy pin `20` |
| RR encoder VCC | Red | Teensy `3.3 V` distribution |
| RR encoder GND | Black | Teensy `GND` distribution |
| RR encoder A | White | Teensy pin `21` |
| RR encoder B | Yellow | Teensy pin `22` |

First test the right side with a low command:

```bash
python3 tools/teensy/teensy_drive_smoke.py --right 30 --left 0 --duration 3
```

Expected telemetry shape:

```text
FR and RR counts change by similar amounts
FL and RL remain near zero
```

If one right-side motor spins opposite to the other, swap that motor's thick
leads at `M1A/M1B` or `M2A/M2B`. If the physical direction is correct but the
encoder sign is opposite, swap that encoder's white/yellow wires.

## Validated Four-Motor Bench Results

These results were captured on 2026-05-27 with all four GR-WM4-V3 motors wired
through the Teensy and two Sabertooth 2x32 controllers.

Final all-forward smooth run:

```text
Command: V 60 60 for 3 s
FL +1596
RL +1627
FR +1673
RR +1584
```

Skid-steer isolation and turning checks:

| Test | Command | Expected Behaviour | Observed Encoder Counts |
|------|---------|--------------------|-------------------------|
| Left only | `V 35 0` | Only FL/RL move | FL `+759`, RL `+791`, FR `0`, RR `0` |
| Right only | `V 0 35` | Only FR/RR move | FL `0`, RL `0`, FR `+790`, RR `+735` |
| Arc left | `V 30 60` | Right side moves faster | FL `+829`, RL `+872`, FR `+1715`, RR `+1634` |
| Arc right | `V 60 30` | Left side moves faster | FL `+1636`, RL `+1667`, FR `+894`, RR `+834` |
| Pivot | `V 40 -40` | Left forward, right reverse | FL `+872`, RL `+901`, FR `-865`, RR `-821` |

This confirms the final bench mapping:

```text
Left command  -> left Sabertooth only  -> FL/RL
Right command -> right Sabertooth only -> FR/RR
```

The firmware ramp is active during these tests. Targets do not jump straight to
the requested value; `cmdL` and `cmdR` step towards the target so the motors
start and stop smoothly.

## Serial Smoke Commands

The current Teensy firmware accepts simple USB serial commands:

```text
V <left> <right>   # side throttle targets, -127..127
X                  # hard stop
E                  # E-stop active: hard stop + latch inhibit
U                  # E-stop released, restart still required
R                  # restart/reset inhibit
Z                  # reset encoders
```

Telemetry is emitted at 10 Hz:

```text
T <ms> <state> <estop> <inhibit> <targetL> <targetR> <cmdL> <cmdR> <FL> <RL> <FR> <RR>
```

Useful bench commands from the repo root:

```bash
# Left side only
python3 tools/teensy/teensy_drive_smoke.py --left 80 --right 0 --duration 3

# Right side only
python3 tools/teensy/teensy_drive_smoke.py --left 0 --right 30 --duration 3

# Simulated E-stop during motion
python3 tools/teensy/teensy_drive_smoke.py --left 80 --right 0 --duration 5 --estop-after 2
```

State values:

| State | Meaning |
|-------|---------|
| `0` | Ready |
| `1` | Driving |
| `2` | Inhibited; restart required |
| `3` | E-stop active |
| `4` | Command timeout |

The tested E-stop behaviour is latched: motion commands are refused while
stopped, `U` only releases the E-stop input, and `R` is required before motion
can resume.

## Jetson Handoff

For the intended rover path, keep the Teensy wired exactly as tested and move
only the USB host connection:

```text
Jetson USB -> Teensy micro-USB
Teensy pin 1 -> left Sabertooth S1
Teensy pin 8 -> right Sabertooth S1
Teensy GND -> both Sabertooth 0V terminals
```

Do not wire the Jetson GPIO header directly to the Sabertooth for this path. The
Jetson should talk to the Teensy over USB serial, and the Teensy should own the
Sabertooth packet serial, encoder counting, ramping, timeout, and E-stop latch.

For a first Jetson test:

1. Leave the motor wiring, encoder wiring, and Sabertooth wiring as-is.
2. Turn the PSU off.
3. Move the Teensy USB cable from the Mac to the Jetson.
4. Confirm the Teensy appears on the Jetson as `/dev/ttyACM*` or
   `/dev/serial/by-id/*`.
5. Run a low-speed serial smoke test before controller teleop.
6. Turn the PSU on only when the serial path is confirmed.

On 2026-05-28, the Teensy appeared on the Jetson as:

```text
/dev/ttyACM0
/dev/serial/by-id/usb-Teensyduino_USB_Serial_19852100-if00
```

Set the Linux serial port to raw/no-echo before talking to the Teensy. Without
this, the Jetson briefly echoed the Teensy boot text back into the firmware on
first open, which triggered the line parser and latched `E` / E-stop. Recovery
was clean with `U`, then `R`.

```bash
stty -F /dev/ttyACM0 115200 raw -echo -echoe -echok -echoctl -echoke -icanon -isig -ixon -ixoff -crtscts
```

Jetson USB-to-Teensy low-level smoke results:

| Test | Command | Observed Encoder Counts |
|------|---------|-------------------------|
| All forward | `V 30 30` | FL `+995`, RL `+1042`, FR `+1056`, RR `+965` |
| Left only | `V 35 0` | FL `+732`, RL `+761`, FR `0`, RR `0` |
| Right only | `V 0 35` | FL `0`, RL `0`, FR `+778`, RR `+711` |
| Pivot | `V 40 -40` | FL `+833`, RL `+861`, FR `-821`, RR `-772` |

This proves the Jetson USB serial path to the Teensy and the Teensy-to-Sabertooth
side split.

## ROS 2 Bridge Path

The `lunabot_drivetrain` ROS bridge now supports the current Teensy USB serial
backend:

```text
serial_protocol: teensy_line
serial_port: /dev/ttyACM0
baud_rate: 115200
```

The bridge sends these commands to the Teensy:

| ROS Bridge Event | Teensy Command |
|------------------|----------------|
| Valid `/cmd_vel_gated` command | `V <left> <right>` |
| Stale command, inhibit, fault, or shutdown | `X` |
| `/safety/estop := true` | `E` |
| E-stop cleared after the bridge recovery wait | `U`, then `R` |

It also reads the Teensy `T ...` telemetry lines and republishes encoder ticks
in the ROS package wheel order: `FL`, `FR`, `RL`, `RR`.

Start the normal bench launch on the Jetson with a low throttle cap:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py max_throttle:=0.25
```

For Xbox testing through the safety path:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py enable_teleop:=true max_throttle:=0.25
```

Validated on the Jetson on 2026-05-28:

| Test | Result |
|------|--------|
| `colcon build --packages-select lunabot_drivetrain` | Passed |
| `colcon test --packages-select lunabot_drivetrain` | `47 passed` |
| ROS bridge launch | `/dev/ttyACM0` opened with `serial_protocol=teensy_line` |
| Safety gate | Opened when drivetrain status became healthy |
| Gated manual command | `/cmd_vel_safe` moved through `/cmd_vel_gated` and changed all four encoder counts |
| Software e-stop | `/safety/estop := true` closed the gate, latched stop, held encoder counts steady, then recovered after `/safety/estop := false` |
| Xbox controller | Detected as `Xbox Series X Controller` on `/dev/input/js0`; commands reached `/cmd_vel_gated` and encoder counts changed |

Manual gated command sample:

```text
Command: /cmd_vel_safe linear.x = 0.06 for 2 s, max_throttle = 0.25
Telemetry after command: FL 622, FR 670, RL 665, RR 601
Status: READY, no fault, no e-stop, controllers online
```

E-stop sample:

```text
Before E-stop: FL 622, FR 670, RL 665, RR 601
After motion then E-stop: FL 1413, FR 1519, RL 1505, RR 1362
0.8 s later while E-stop active: counts unchanged
After release/restart wait: READY, no fault, gate open
```

Xbox teleop sample:

```text
Controller: Xbox Series X Controller
Path: /joy -> /cmd_vel_teleop -> /cmd_vel_safe -> /cmd_vel_gated -> drivetrain_bridge
Observed: angular and forward commands reached /cmd_vel_gated; final telemetry
          showed FL 5263, FR 2017, RL 5460, RR 1726 with no drivetrain fault.
```

For immediate bench teleop, plug the Xbox controller into the Jetson. For the
competition-shaped setup, the controller can live on the operator laptop and use
the browser Gamepad bridge served by the Jetson. The browser sends HTTPS
commands over rover Wi-Fi to the Jetson, the Jetson publishes `/cmd_vel_safe`,
and the normal velocity gate and drivetrain bridge still own the ROS side.

See `docs/foxglove/README.md` for the browser Gamepad bridge launch command,
HTTPS certificate note, and operator URL.
