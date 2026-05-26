# Gimson Robotics GR-WM4-V3 — Datasheet

## Overview

| Field | Value |
|-------|-------|
| **Manufacturer** | Gimson Robotics |
| **Part Number** | GR-WM4-V3 |
| **Role on Rover** | Drivetrain — four-wheel drive locomotion on the motive power rail |
| **Power Domain** | 🔴 Motive (22.2 V nominal) |
| **Qty on Rover** | 4 |
| **Product Page** | https://gimsonrobotics.co.uk/collections/all |
| **Weight** | ~900 g each / ~3600 g total (estimated — high-torque geared motor with encoder) |

---

## Electrical Specifications

| Parameter | Value |
|-----------|-------|
| Rated voltage | 24 V DC nominal |
| Operating voltage range | Compatible with 22.2 V motive rail (fully charged: 25.2 V) |
| No-load speed | 37 RPM |
| Rated speed (at 91.8 kg·cm load) | 28 RPM |
| Stall current | **22.6 A** |
| Peak current (gearbox limit) | **13 A** — hard limit imposed by gearbox bearing rating |
| Motor connector | Bullet terminals / 6-pin connector |

> **Operational note:** The gearbox's mechanical limit is 13 A but the Sabertooth 2×32 is
> software-configured to limit each channel to **10 A** — protecting the gearbox while capping
> total drivetrain peak at 40 A (4 × 10 A), giving a 3× safety factor over the 120 A battery rating.

### INNEX1 Power Budget

| Scenario | Current per Motor | Total (4 motors) | Battery Safety Factor |
|----------|--------------------|-------------------|----------------------|
| Normal driving | ~3–5 A | ~12–20 A | **6–10×** (120 A / ~12–20 A) |
| Peak demand (Sabertooth software-limited) | 10 A | **40 A** (4 × 10 A software cap) | **3×** (120 A / 40 A) |

> Driving and excavation must **never run simultaneously** — see main battery datasheet.

---

## Mechanical Specifications

| Parameter | Value |
|-----------|-------|
| Gearbox type | Single-stage worm-and-wheel |
| Gear ratio | ~64:1 |
| Output torque (rated) | 91.8 kg·cm (9.0 N·m) |
| Output torque (max, bearing limited) | 222 kg·cm (21.8 N·m) |
| Self-locking | **Yes** — worm gear holds position when power is removed |
| IP rating | **IP30** — not rated for outdoor/exposed use |
| Duty cycle (rated load) | **40%** — 60 s on, 90 s minimum off |

> **Thermal caution:** At rated load, maximum 40% duty cycle must be respected. Running
> continuously at full load will overheat the motor windings. The rover's drive profiles must
> enforce rest periods during sustained excavation or high-torque manoeuvres.

### Mounting Notes

- Mount with the output shaft horizontal or vertical — worm gearbox orientation is flexible
- Secure with M-series bolts through the gearbox flange; use thread-locking compound for vibration resistance on rough terrain
- The self-locking property means the rover holds position on slopes when stopped — no braking mechanism required for drivetrain
- IP30 rating means **no exposure to regolith dust ingress** — enclose or shield the motor body within the chassis

---

## Encoder Specifications

| Parameter | Value |
|-----------|-------|
| Encoder type | Quadrature Hall-effect (SS460S) |
| Resolution (motor shaft) | 12 PPR |
| Resolution (output shaft) | **720 pulses/rev** (12 PPR × ~64:1 gear ratio, both channels) |
| Encoder supply voltage | 5 V (SS460S operates down to 3 V minimum) |
| Signal voltage | 5 V logic |
| Output channels | A + B (quadrature) |
| Connector | 4-wire (VCC, GND, CH-A, CH-B) |

### INNEX1 Encoder Wiring

| Signal | Wire Colour | Connection |
|--------|------------|------------|
| VCC | Red | Teensy 4.1 3.3 V pin (via WAGO 2-in-4-out) |
| GND | Black | Teensy GND (star topology, separate WAGO output from motor controllers) |
| CH-A | Blue | Teensy GPIO directly (see pin table below) |
| CH-B | Yellow | Teensy GPIO directly (see pin table below) |

> **No level shifter required:** The SS460S Hall sensors operate down to 3 V. Powering the
> encoders from the Teensy 3.3 V rail means signals swing 0–3.3 V — directly safe for Teensy
> GPIO with no level shifting. Encoders are completely off the BD-02 compute rail.
> Two Teensy 3.3 V pins feed a WAGO 2-in-4-out block to supply all four encoder VCC wires.

### Teensy 4.1 Pin Assignment (Encoders)

| Motor | CH A Pin | CH B Pin |
|-------|----------|----------|
| Front-Left (FL) | 15 | 16 |
| Rear-Left (RL) | 17 | 18 |
| Front-Right (FR) | 19 | 20 |
| Rear-Right (RR) | 21 | 22 |

---

## Motor Controller Interface

These motors are driven by two **Sabertooth 2×32** motor controllers (2 motors per controller,
one per side of the rover).

| Parameter | Value |
|-----------|-------|
| Controller | Sabertooth 2×32 |
| Communication | Packetized serial (9600 baud, address 128) |
| Sabertooth #1 (Left: FL + RL) | Teensy Pin 0 (UART1 TX) |
| Sabertooth #2 (Right: FR + RR) | Teensy Pin 7 (UART2 TX) |
| Jetson role | High-level commands only — talks to Teensy over USB serial |
| Hardware ramping | `setSaberRamping(20)` — configured once at startup |
| Power-on sequence | Always power Jetson and Teensy before applying 22.2 V to Sabertooth |

> All motor I/O is handled by the Teensy 4.1. The Jetson sends velocity targets over USB serial
> to the Teensy, which translates them into Sabertooth packetised serial commands. The Sabertooth
> hardware ramping (`setSaberRamping(20)`) handles smooth acceleration and deceleration automatically.

---

## ROS 2 Software Integration

| Item | Value |
|------|-------|
| ROS 2 Package | `lunabot_drivetrain` |
| Command topic | `/cmd_vel_safe` (`geometry_msgs/msg/Twist`) |
| Encoder feedback | Jetson GPIO (quadrature, via level shifter) |
| Launch file | `drivetrain_bench.launch.py` |

```bash
# Build and launch drivetrain
cd ~/innex1-rover
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py max_throttle:=0.2

# Send a test velocity command
ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

---

## Key Notes & Constraints

- **Stall current vs software limit:** Stall current is 22.6 A electrically but the gearbox
  bearing is limited to 13 A peak. The Sabertooth is software-configured to 10 A/channel —
  protecting the gearbox with margin. Total drivetrain peak is 40 A (4 × 10 A, SF 3×).
- **40% duty cycle:** At rated load, 60 s on / 90 s off is mandatory. Mission planning must account
  for this — continuous full-throttle driving on soft regolith simulant will approach rated load quickly.
- **IP30 — no dust protection:** The motor body has no sealing against lunar regolith simulant.
  Shield the motors within the chassis or add a simple cover. Fine regolith analogue can jam the
  gearbox over a competition run.
- **Self-locking on slopes:** The worm gearbox is self-locking in most conditions, which is an
  advantage for hill holding but means back-driving for repositioning is not possible without power.
- **No level shifter needed:** Encoders are powered from Teensy 3.3 V — signals swing 0–3.3 V
  and connect directly to Teensy GPIO. Do not power encoders from 5 V or they will output 5 V
  signals that could damage Teensy GPIO pins.
- **Power-on order:** Always power the Jetson and microcontroller before applying 24 V to the
  Sabertooth. Reverse order can cause the Sabertooth to interpret floating signal lines as commands.

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-23 | eniomecaj | Initial datasheet |
| 2026-05-24 | eniomecaj | Updated peak demand to 40 A (10 A/ch × 4, SF 3×); encoder pin table confirmed |
