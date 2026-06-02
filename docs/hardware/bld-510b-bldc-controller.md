# BLD-510B BLDC Motor Driver — Datasheet

## Overview

| Field | Value |
|-------|-------|
| **Manufacturer** | STEPPERONLINE |
| **Model** | BLD-510B |
| **Role on Rover** | Brushless DC motor driver — excavation motor (57BLR50) |
| **Power Domain** | 🔴 Motive (22.2 V from WUPP fuse block slot 5, 10 A blade fuse) |
| **Qty on Rover** | 1 |
| **Weight** | ~200 g (estimated from dimensions 118 × 75.5 × 34 mm) |

---

## Electrical Specifications

| Parameter | Value |
|-----------|-------|
| **Supported input voltages** | 12 / 24 / 36 / 48 V DC |
| **INNEX-1 supply** | **22.2 V motive rail** (≈24 V nominal — slightly below 24 V rated) |
| Continuous output current @ 24 V | **8.3 A** |
| Continuous output current @ 12 V | 10 A |
| Peak current | **15 A** |
| Rated power @ 24 V | 200 W |
| PWM chopper frequency | 20 kHz |
| Operating temperature | 0 – 45 °C |
| Dimensions | 118 × 75.5 × 34 mm |
| Cooling | Passive heatsink (active airflow recommended in enclosure) |

> **22.2 V vs 24 V:** The INNEX-1 motive rail at 22.2 V is slightly below the 24 V nominal
> rating. The driver operates correctly at this voltage; output power is marginally lower but
> the 57BLR50 motor is within limits. The motive rail at full charge is ~25.2 V (6S LiPo),
> which is also within the 24 V continuous range. Battery protect cutoff will reduce voltage
> during discharge — monitor motor performance at low battery.

---

## Physical Dimensions

- 118 × 75.5 × 34 mm
- Passive heatsink — ensure airflow inside chassis enclosure during sustained excavation runs

---

## Terminal Reference

### Power Input

| Terminal | Description |
|----------|-------------|
| V+ | 22.2 V motive rail (from WUPP slot 5, 10 A fuse) |
| GND | Motive domain ground |

### Motor Phase & Hall Outputs

| Terminal | Description |
|----------|-------------|
| MA | Motor Phase A |
| MB | Motor Phase B |
| MC | Motor Phase C |
| GND | Motor GND (common) |
| HA | Hall sensor A input |
| HB | Hall sensor B input |
| HC | Hall sensor C input |
| +5V | Hall sensor power supply (5 V) |

> Connect MA/MB/MC to the 57BLR50 phase wires (Orange/Green/Brown = Phase U/V/W — verify
> with 57blr50-excavation-motor.md). If rotation is wrong, swap any two phase wires.
> Connect HA/HB/HC to the Hall sensor outputs (Yellow/White/Blue = Hall A/B/C).

### Control Signal Terminal (8-pin)

| Pin | Name | Description |
|-----|------|-------------|
| 1 | GND | Signal ground — connect to Teensy GND |
| 2 | F/R | Direction: open/high = CW (facing shaft); connect to GND = CCW |
| 3 | EN | Enable: V2.0 — connect to GND = run, open = stop |
| 4 | BK | Brake: connect to GND = fast brake stop; open = run |
| 5 | SV | Speed reference: 0–5 V analogue OR 1–2 kHz PWM (5 V amplitude) |
| 6 | PG | Speed pulse output (open-collector) — needs 3–10 kΩ pull-up to 5 V |
| 7 | ALM | Alarm output (open-collector, active LOW) — needs 3–10 kΩ pull-up to 5 V |
| 8 | +5V | 5 V supply output (do not use to power Teensy — use Jetson USB) |

---

## INNEX-1 Control Wiring (Teensy 4.1)

| BLD-510B Pin | Signal | Teensy 4.1 Pin | Notes |
|-------------|--------|----------------|-------|
| GND | Signal GND | GND | Shared logic ground |
| SV | Speed PWM | **Pin 6** | 1–2 kHz PWM, 3.3 V amplitude (see note) |
| F/R | Direction | **Pin 13** | LOW = CCW, HIGH/open = CW |
| EN | Enable | **Pin 14** | HIGH = stop (V2.0 logic), LOW = run |
| BK | Brake | **TBD** | LOW = brake; use GPIO output |
| PG | Speed feedback | **Pin 31** | Open-collector — add 10 kΩ pull-up to 3.3 V |
| ALM | Alarm | **Pin 32** | Open-collector — add 10 kΩ pull-up to 3.3 V |

> **SV PWM — no external RC filter needed:** The BLD-510B SV pin accepts PWM directly (1–2 kHz
> from Teensy Pin 6). No external RC filter is required — the driver handles the PWM internally.
> Teensy outputs 3.3 V PWM which gives ~66% of rated speed at full duty cycle; adjust the R-SL
> potentiometer to scale as needed. See `teensy-4.1-microcontroller.md` for confirmed pin assignments.

---

## Speed Control — PWM via SV Pin

The Teensy sends a **1–2 kHz PWM signal** to the SV pin:
- Duty cycle 0% → motor speed 0 (stops below 0.3 V equivalent)
- Duty cycle 100% → motor speed proportional to V_HIGH (3.3 V from Teensy)
- Motor speed scales linearly with duty cycle

> To use the full speed range with 3.3 V logic, set the built-in R-SL potentiometer to 100%
> (maximum, clockwise). The maximum motor speed is then set by the internal jumpers.

---

## Maximum Speed Jumpers (Internal — Set Before Assembly)

Internal jumpers 1–4 select max speed based on motor pole pairs. The **57BLR50** motor pole pair
count must be confirmed from the motor datasheet — see `57blr50-excavation-motor.md`.

Default factory setting: **7000 RPM** for 1 pole pair, **3500 RPM** for 2 pole pairs (jumper 1=1, 2=0, 3=0, 4=0).

> Set the maximum speed jumpers before installing the driver in the chassis. Disassemble the
> housing to access the jumpers. Confirm motor pole pairs from the 57BLR50 datasheet.

---

## EN / F/R Logic (Version-Dependent)

> ⚠️ Two firmware versions exist: **V2.0** and **V2.4** with opposite EN logic.

| Version | EN connected to GND | EN open |
|---------|-------------------|---------|
| **V2.0** | Motor **runs** | Motor **stops** |
| **V2.4** | Motor **stops** | Motor **runs** |

**Verify version label on driver before writing Teensy firmware.** Default assumption in
INNEX-1 code should be V2.0 (connect EN to GND = run). If behaviour is reversed, invert EN logic.

---

## Protection Functions

| Protection | Behaviour |
|-----------|-----------|
| Over-current | Driver stops; red LED on |
| Over-voltage | Driver stops; red LED on |
| Under-voltage | Driver stops; red LED on |
| Over-temperature | Driver stops; red LED on |
| Hall signal fault | Driver stops; red LED on |
| Stall | Driver stops |

To clear an alarm: disconnect EN from GND (stop command) or cycle power.
The ALM output goes LOW during any alarm — Teensy should monitor this pin.

---

## On-Board Indicators

| Indicator | Colour | Meaning |
|-----------|--------|---------|
| P/A — Green | Green | Motor running (always on when operating) |
| P/A — Red | Red | Alarm / fault (always on during fault) |

---

## Key Rules & Notes

- **PG and ALM are open-collector** — if you intend to monitor speed feedback (PG) or fault
  alarms (ALM), fit 10 kΩ pull-up resistors to 3.3 V at Teensy Pins 31 and 32. If PG/ALM
  monitoring is not used, pull-ups can be omitted and the pins left unconnected.
- **Check EN firmware version before first power-on** — wrong EN polarity causes motor to
  run immediately at power-up, before the Teensy asserts control.
- **Never change F/R direction while motor is running** — always bring motor to stop first
  (EN stop command), then change F/R, then restart. Changing direction mid-run can damage driver.
- **BK (brake) for fast stop only** — braking places electrical stress on the system.
  Use natural stop (EN) for routine stops; reserve BK for emergency use.
- **Fuse:** 10 A blade fuse in WUPP slot 5. At 22.2 V the driver draws up to 8.3 A continuous;
  a 10 A fuse provides protection without nuisance tripping.
- **Heatsink ventilation** — mount with heatsink fins vertical if possible; ensure chassis
  enclosure has airflow around the driver during sustained operation.
- **Hall sensor +5V** — use the driver's own +5V terminal to power Hall sensors. Do not share
  this with other logic supplies.
- **Power-up sequence** — apply 22.2 V only after Teensy has asserted EN = stop (open).
  Prevents motor running before software is ready.

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-24 | eniomecaj | Initial datasheet — sourced from BLD-510B User Manual V1 (STEPPERONLINE, 2024) |
| 2026-05-24 | eniomecaj | Corrected pin assignments (F/R↔EN, PG→31, ALM→32); RC filter removed (not needed); PG/ALM pull-ups made optional |
  