# Cytron MDD10A — Dual Channel DC Motor Driver Datasheet

## Overview

| Field | Value |
|-------|-------|
| **Manufacturer** | Cytron Technologies |
| **Model** | MDD10A (RB-Cyt-153, Rev2.0) |
| **Role on Rover** | Dual-channel PWM motor driver — linear actuators |
| **Power Domain** | 🔴 Motive (see per-unit detail below) |
| **Qty on Rover** | 2 |
| **Weight** | ~100 g each / ~200 g total (estimated — 84.5 × 62 mm PCB) |

| Unit | Power Input | Controls |
|------|-------------|----------|
| **Cytron MDD10A #1** | 22.2 V directly from WUPP fuse block (slot 3) | 50mm TRU Components actuators ×2 |
| **Cytron MDD10A #2** | 12 V from DollaTek buck converter (fuse block slot 4 → buck → Cytron) | 250mm DCHOUSE actuators ×2 |

---

## Electrical Specifications

| Parameter | Value |
|-----------|-------|
| **Motor supply voltage (VPWR)** | **5 – 30 V** (Rev2.0) |
| **Cytron #1 supply** | **22.2 V** ✅ (50mm actuators rated 24 V) |
| **Cytron #2 supply** | **12 V** ✅ (250mm actuators rated 12 V, via DollaTek buck) |
| Continuous output current (per channel) | **10 A** |
| Peak output current (per channel, ≤10 s) | **30 A** |
| Logic input high (PWM, DIR) | 3.0 – 5.5 V |
| Logic input low | 0 – 0.5 V |
| Max PWM frequency | **20 kHz** |
| Dimensions | 84.5 × 62 mm |

> **Logic compatibility:** 3.0 V minimum for logic HIGH — Teensy 4.1 (3.3 V logic) is directly
> compatible. No level shifter required.

---

## Control Interface (Sign-Magnitude PWM)

INNEX-1 uses **sign-magnitude PWM** mode: one PWM signal sets speed, one DIR signal sets direction.

| PWM | DIR | Motor Output | Effect |
|-----|-----|--------------|--------|
| LOW | X | A = Low, B = Low | Stop (coast) |
| HIGH | LOW | A = High, B = Low | Forward |
| HIGH | HIGH | A = Low, B = High | Reverse |

> **Locked-antiphase alternative:** Connect PWM pin to 3.3 V and feed the DIR pin with a PWM
> signal. 50% duty = stop, <50% = one direction, >50% = other. INNEX-1 does NOT use this mode.

---

## Pin Connectors

### Control Header (5-pin, 2510 connector)

| Pin | Name | Description |
|-----|------|-------------|
| 1 | GND | Logic ground — connect to Teensy GND |
| 2 | PWM2 | Speed control, Motor 2 (TTL PWM, not RC PWM) |
| 3 | DIR2 | Direction, Motor 2 (HIGH = reverse) |
| 4 | PWM1 | Speed control, Motor 1 |
| 5 | DIR1 | Direction, Motor 1 |

### Power Terminal Block (6-position screw terminal)

| Position | Name | Description |
|----------|------|-------------|
| 1 | M1B | Motor 1 Output B |
| 2 | M1A | Motor 1 Output A |
| 3 | POWER + | Motor supply positive (from fuse block / buck converter) |
| 4 | POWER − | Motor supply negative (GND) |
| 5 | M2A | Motor 2 Output A |
| 6 | M2B | Motor 2 Output B |

---

## INNEX-1 Wiring

### Cytron MDD10A #1 — 50mm Actuators (22.2 V)

| Cytron Terminal | INNEX-1 Connection |
|----------------|--------------------|
| POWER + | WUPP fuse block slot 3 (15 A blade fuse) |
| POWER − | Motive domain GND bus |
| M1A / M1B | TRU Components 50mm Actuator #1 (Brown +, Blue −) |
| M2A / M2B | TRU Components 50mm Actuator #2 (Brown +, Blue −) |
| GND (header) | Teensy GND |
| PWM1 | Teensy pin **2** |
| DIR1 | Teensy pin **9** |
| PWM2 | Teensy pin **3** |
| DIR2 | Teensy pin **10** |

### Cytron MDD10A #2 — 250mm Actuators (12 V via DollaTek buck)

| Cytron Terminal | INNEX-1 Connection |
|----------------|--------------------|
| POWER + | DollaTek buck output + (set to 12 V) |
| POWER − | DollaTek buck output − (GND) |
| M1A / M1B | DCHOUSE 250mm Actuator #3 (Red +, Black −) |
| M2A / M2B | DCHOUSE 250mm Actuator #4 (Red +, Black −) |
| GND (header) | Teensy GND |
| PWM1 | Teensy pin **4** |
| DIR1 | Teensy pin **11** |
| PWM2 | Teensy pin **5** |
| DIR2 | Teensy pin **12** |

---

## On-board LEDs

| LED | Colour | Meaning |
|-----|--------|---------|
| Power | Green | Board is powered |
| M1A | Red | Current flowing from M1A → M1B (motor 1 CW) |
| M1B | Red | Current flowing from M1B → M1A (motor 1 CCW) |
| M2A | Red | Current flowing from M2A → M2B (motor 2 CW) |
| M2B | Red | Current flowing from M2B → M2A (motor 2 CCW) |

The direction LEDs are useful for confirming correct actuator polarity during bench testing.

---

## Key Rules & Notes

- **10A continuous, 30A peak** — the 50mm actuators draw up to 1.6 A each (well within limit).
  The 250mm actuators draw up to 3 A each; combined max is 6 A — within the 10 A rating.
- **22.2 V is within the 30 V max for Cytron #1** ✅ — 50mm actuators rated 24 V, so the
  motor voltage slightly exceeds actuator rating; keep duty cycles ≤ 100% and monitor heat.
  Consider capping PWM to ~90% to approximate 20 V if required.
- **Cytron #2 must be fed from the DollaTek buck only** — do not connect directly to the 22.2 V
  motive rail; 250mm actuators are rated 12 V and will be damaged.
- **Not for RC PWM** — the PWM input accepts TTL-level digital PWM from the Teensy, not the
  1–2 ms RC servo signal type.
- **Use battery (motive LiPo), not bench power supply** — regenerative current from actuators
  can trip the overcurrent protection on a switching supply. Always test with the motive LiPo.
- **Test buttons on board** — each motor channel has a physical test button (M1A, M1B, M2A, M2B)
  for bench verification without microcontroller. Useful for polarity checks before software.
- **Logic ground must be shared** — connect the GND pin on the control header to Teensy GND.
  A floating logic ground will cause erratic behaviour.

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-24 | eniomecaj | Initial datasheet — sourced from MDD10A User Manual V2.0 (Cytron Technologies, June 2017) |
