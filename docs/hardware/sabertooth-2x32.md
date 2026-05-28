# Sabertooth 2×32 Motor Driver — Datasheet

## Overview

| Field | Value |
|-------|-------|
| **Manufacturer** | Dimension Engineering |
| **Model** | Sabertooth 2×32 |
| **Role on Rover** | Dual-channel brushed DC motor driver — drive train |
| **Power Domain** | 🔴 Motive (22.2 V input from WUPP fuse block, no individual blade fuse) |
| **Qty on Rover** | 2 |
| **INNEX-1 Assignment** | #1 = Left drivetrain (FL + RL motors); #2 = Right drivetrain (FR + RR motors) |

---

## Electrical Specifications

| Parameter | Value |
|-----------|-------|
| **Input voltage (B+ to B−)** | **6.0 – 33.6 V** |
| **INNEX-1 supply** | **22.2 V motive rail** ✅ (within range) |
| Continuous output current (per channel) | **32 A** |
| Peak output current (per channel) | **64 A** |
| **INNEX-1 software current limit** | **≤ 10 A / channel** (⚠️ see safety note below) |
| Output voltage (M1, M2) | ±95% of input voltage |
| 5V logic output | 5 V @ 1 A max |
| Power outputs (P1, P2) | Open collector, 8 A sink max each |
| Signal input voltage (S1, S2, A1, A2) | 0 – 5 V (3.3 V logic compatible — no level shifter needed) |
| Switching frequency | 30 kHz (silent — no motor whine) |
| Operating temperature | −20 °C to +70 °C (derates above 40 °C ambient) |

> **Current derating:** At 40 °C ambient the full 32 A/channel is available. At 70 °C ambient
> continuous rating drops to 10 A/channel. INNEX-1 software limits to ≤10 A/channel regardless.

---

## ⚠️ Safety: Software Current Limit — REQUIRED for INNEX-1

The motive domain trunk fuse is **40 A ANL** (protecting the entire motive rail). There is
**no blade fuse between the fuse block and the Sabertooth** — the Sabertooth's own 64 A peak
capability would allow cable damage before the ANL fuse trips.

**The Teensy 4.1 firmware MUST configure the Sabertooth current limit to ≤ 10 A per channel**
before commanding any motion. This is enforced via the DEScribe PC software or packet serial
commands. Without this limit, the motor wiring may be at risk.

See `software-team-notes.md` and `fuses.md` for the full fuse rationale.

---

## Physical Specifications

| Parameter | Value |
|-----------|-------|
| Dimensions | 70 × 90 × 26 mm (2.75 × 3.5 × 1.0 in) |
| Weight | 125 g (4.5 oz) |
| Recommended motor wire | 10–12 AWG |
| Recommended battery wire | 10 AWG (INNEX-1 uses 10 AWG ✅) |

---

## Terminal Wiring Reference

| Terminal | Description | Voltage | Current |
|----------|-------------|---------|---------|
| **B+** | Power input positive | 6 – 33.6 V | 64 A peak |
| **B−** | Power input negative | 0 V (GND) | 64 A peak |
| **M1A / M1B** | Motor 1 output | 0 – 33.6 V | 32 A / 64 A peak |
| **M2A / M2B** | Motor 2 output | 0 – 33.6 V | 32 A / 64 A peak |
| **0V** | Logic ground (tied internally to B−) | 0 V | 2 A max |
| **5V** | Regulated 5 V output | 5 V | 1 A max |
| **S1** | Serial RX (main signal input) | 0 – 5 V | 1 mA |
| **S2** | Serial TX (main signal input / indicator output) | 0 – 5 V | 1 mA in / 20 mA out |
| **A1** | Auxiliary input 1 | 0 – 5 V | 1 mA |
| **A2** | Auxiliary input 2 / indicator output | 0 – 5 V | 1 mA in / 20 mA out |
| **P1 / P2** | Power outputs (open collector sink) | 0 – 33.6 V | 8 A each |

---

## INNEX-1 Wiring

| Sabertooth Terminal | INNEX-1 Connection |
|--------------------|--------------------|
| B+ | WUPP fuse block (motive rail 22.2 V) |
| B− | Motive domain GND bus |
| M1A / M1B | Drive motor 1 (e.g. Front-Left or Front-Right) |
| M2A / M2B | Drive motor 2 (e.g. Rear-Left or Rear-Right) |
| S1 | Teensy 4.1 TX (UART) |
| S2 | Teensy 4.1 RX (optional — needed for telemetry/readback) |
| 0V | Common GND with Teensy / logic domain |

**Teensy UART pin assignments (from `teensy-4.1-microcontroller.md`):**
- Sabertooth #1: Teensy pin 1 (TX1) to S1, optional pin 0 (RX1) from S2 — Left drivetrain
- Sabertooth #2: Teensy pin 8 (TX2) to S1, optional pin 7 (RX2) from S2 — Right drivetrain

**Motor channel assignment:**

| Controller | Channel | Motor |
|------------|---------|-------|
| Sabertooth #1 left | M1A / M1B | Front-left |
| Sabertooth #1 left | M2A / M2B | Rear-left |
| Sabertooth #2 right | M1A / M1B | Front-right |
| Sabertooth #2 right | M2A / M2B | Rear-right |

The full drivetrain was bench-tested on 2026-05-27 with FL/RL on the left
controller and FR/RR on the right controller. Final checks confirmed independent
skid-steer control:

```text
V 35 0 -> only FL/RL moved
V 0 35 -> only FR/RR moved
V 30 60 -> right side moved about twice as far as left
V 40 -40 -> left positive, right negative pivot
```

Do not tie the two `S1` inputs together. Left `S1` goes only to Teensy pin `1`;
right `S1` goes only to Teensy pin `8`.

---

## Serial Control Mode

The Sabertooth is controlled by the Teensy 4.1 over **TTL UART (plain text or packet serial)**.

### DIP Switch Settings for Serial Mode

| DIP Switch | Position | Function |
|------------|----------|----------|
| 1 | — | Baud rate (see below) |
| 2 | — | Baud rate (see below) |
| 3 | OFF | Battery protect mode (LiPo auto-detect) |
| **4** | **ON** | **Packet / plain text serial mode** |
| **5** | **ON** | **Packet serial address 128** |
| **6** | **ON** | **Emergency stops disabled** (A1/A2 not used as E-stop) |

> DIP 6 ON = emergency stops disabled (current INNEX-1 setup — software stop via Teensy).
> To enable hardware E-stop via A1/A2: set DIP 6 OFF, then wire the E-stop contacts so that
> A1 and A2 are normally held at 5 V (via Sabertooth's own 5 V output) and drop to 0 V when
> the E-stop is pressed. With DIP 6 OFF: A1/A2 HIGH (5 V) = motors enabled; A1/A2 LOW (0 V)
> = both channels cut immediately in hardware, before any software response.

### Baud Rate (DIP 1 & 2, switch 4 ON)

| DIP 1 | DIP 2 | Baud Rate |
|-------|-------|-----------|
| OFF | OFF | 2400 |
| ON | OFF | 9600 ✅ (default, recommended) |
| OFF | ON | 19200 |
| ON | ON | 38400 |

**Use 9600 baud (DIP1 ON, DIP2 OFF) for INNEX-1.**

### Plain Text Serial Commands (key examples)

```
M1: 2047\r\n       → Motor 1 full forward
M2: -2047\r\n      → Motor 2 full reverse
M1: 0\r\n          → Motor 1 stop
M1: getc\r\n       → Read motor 1 current (tenths of amp)
M1: getb\r\n       → Read battery voltage (tenths of volt)
M1:shutdown\r\n    → Hard brake motor 1
M1:startup\r\n     → Resume normal operation
```

Command range: −2047 to +2047. Positive = forward, negative = reverse.

---

## Motor Connection Direction

| Wire to M1A | Wire to M1B | Effect |
|-------------|-------------|--------|
| Motor + (Red) | Motor − (Black) | Forward when positive command |
| Motor − (Black) | Motor + (Red) | Reverse when positive command |

> If drive direction is wrong, swap M1A/M1B at the Sabertooth terminal. Do not invert
> in software first — establish physical forward in wiring, then calibrate software.

---

## Key Rules & Notes

- **Power up sequence:** Always power Jetson + Teensy **before** applying 22.2 V to the motive rail.
  Floating signal lines on Sabertooth power-on can be interpreted as drive commands.
- **Software current limit ≤ 10 A/channel is MANDATORY** — no hardware fuse protects the branch.
- **22.2 V is within the 33.6 V max** — 6S LiPo fully charged ~25.2 V is also within spec ✅
- **Battery protect mode (DIP 3 OFF):** Sabertooth will auto-detect 6S LiPo (6 cells) and stop motors
  when voltage is depleted. Verify cell count is detected correctly at startup (Status LED blink pattern).
- **3.3 V logic compatible:** Teensy 4.1 is 3.3 V — no level shifter required on S1/S2.
- **0V must be shared with Teensy GND** for serial communication — connect logic grounds.
- **Regenerative braking:** Returns energy to battery during braking — do not use with non-rechargeable
  batteries. 6S LiPo is correct for this.
- **Do not exceed 10 AWG on battery leads** — Sabertooth terminal max is 10 AWG.
- **DEScribe software** (Windows) used to set current limits, configure modes, update firmware.
  Connect via Micro USB. No battery/motors needed for configuration.

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-24 | eniomecaj | Initial datasheet — sourced from Sabertooth 2×32 User Manual (Dimension Engineering) |
| 2026-05-24 | eniomecaj | Clarified A1/A2 E-stop wiring explanation |
