# TRU Components TC-13492780 — Linear Actuator 50mm Datasheet

## Overview

| Field | Value |
|-------|-------|
| **Manufacturer** | TRU Components (Conrad Electronic SE) |
| **Part Number (MPN)** | TC-13492780 |
| **Datasheet Item No.** | 3373195 |
| **EAN** | 4064161472003 |
| **Order Code** | 18-2816 |
| **Role on Rover** | Linear actuation — deployment / adjustment mechanism |
| **Power Domain** | 🔴 Motive (24 V from motive rail via Cytron MDD10A) |
| **Qty on Rover** | 2 |
| **Price** | £67.75 (pair) |
| **Supplier** | TRU Components |
| **Compliance** | RoHS |
| **Weight** | ~200 g each / ~400 g total (estimated) |

---

## Electrical Specifications

| Parameter | Value |
|-----------|-------|
| Input voltage | **24 V DC** |
| No-load speed | 16 ± 10% mm/s |
| Rated current | **1.6 A** |
| Rated load capacity | **500 N** |
| Duty cycle | **10% — 2 min on / 18 min off** |
| Hall effect sensor | **Yes** (included, unused on INNEX-1) |
| Protection class | IP65 |
| Operating temperature | −20 °C to +60 °C |

> **Duty cycle is critical:** Maximum 2 minutes continuous operation followed by 18 minutes off.
> Exceeding this will overheat the motor winding. Mission profiles must enforce timed actuation
> with adequate rest between moves.

---

## Mechanical Specifications

| Parameter | Value |
|-----------|-------|
| Stroke length | **50 mm** |
| Minimum installation length | 155 ± 3 mm |
| Cable length | 900 ± 20 mm |
| Limit switch | Inner (built-in, both ends of travel) |
| Thrust | 500 N rated |

> The inner limit switch cuts motor current at both ends of travel — the actuator will stop
> automatically at full extend and full retract without needing software end-stop detection.

---

## Wire Colours & INNEX-1 Connections

The actuator ships with **6 wires**. INNEX-1 uses only the 2 motor power wires — the encoder
is not connected as the Cytron MDD10A runs open-loop.

| Wire Colour | Function | INNEX-1 Connection |
|-------------|----------|-------------------|
| **Brown** | Motor + | Cytron MDD10A M1A (or M2A) |
| **Blue** | Motor − | Cytron MDD10A M1B (or M2B) |
| Red | Encoder +5V | **Not connected** |
| Black | Encoder GND | **Not connected** |
| Yellow | Encoder CH-A | **Not connected** |
| White | Encoder CH-B | **Not connected** |

> **Direction reversal:** If the actuator extends when it should retract, swap Brown and Blue at
> the Cytron M1A/M1B terminals. Do not invert direction in software first.

> **Unused encoder wires:** Tape or heat-shrink the Red, Black, Yellow, and White wires
> individually to prevent shorts. Do not leave bare ends loose near the motive rail.

---

## Motor Controller Interface

| Parameter | Value |
|-----------|-------|
| Controller | Cytron MDD10A (dual channel) |
| Actuators per controller | 2 (one per channel) |
| Control type | **Open-loop — timed PWM commands, no position feedback** |
| Teensy PWM pins | Actuators 1 & 2: Pins 2, 3 / Actuators 3 & 4: Pins 4, 5 |
| Teensy DIR pins | Actuators 1 & 2: Pins 9, 10 / Actuators 3 & 4: Pins 11, 12 |
| Fuse (motive fuse block) | **15 A blade** per Cytron MDD10A slot |

> Both actuators on a single Cytron channel share a 15 A fuse. Peak draw per actuator is 1.6 A
> so two actuators = 3.2 A combined — well within the 10 A Cytron channel rating. Note: a 10 A
> fuse would give tighter overcurrent protection for the 3.2 A load; 15 A is the installed size.

---

## Key Rules & Notes

- **2 min on / 18 min off** — strictly enforced duty cycle. Do not run continuously.
- **IP65 rated** — sealed against dust and water jets; suitable for regolith environment.
- **Built-in limit switches** cut power at end of travel — no software end-stops needed, but
  software should still time out commands to avoid stalling against the limit switch repeatedly.
- **Encoder unused** — open-loop control only. If position feedback is needed in future, the
  encoder wires (Red/Black/Yellow/White) are available but currently disconnected.

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-24 | eniomecaj | Initial datasheet — sourced from TRU Components datasheet (item 3373195) and INNEX-1 wiring notes |
| 2026-05-24 | eniomecaj | Removed redundant Teensy voltage note; added fuse sizing note (10 A would give tighter protection) |
