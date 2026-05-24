# 57BLR50-24-01-HG100 — Excavation BLDC Motor Datasheet

## Overview

| Field | Value |
|-------|-------|
| **Manufacturer** | StepperOnline |
| **Part Number** | 57BLR50-24-01-HG100 |
| **Role on Rover** | Excavation — primary digging/regolith collection motor |
| **Power Domain** | 🔴 Motive (22.2 V nominal, compatible — see note) |
| **Qty on Rover** | 1 |
| **Price** | £46.79 |
| **Supplier** | StepperOnline |
| **Drawing Date** | 12.8.2023 |
| **Weight** | ~800 g (estimated — 57 mm BLDC with HG100 100:1 gearbox) |

---

## Electrical Specifications

| Parameter | Value |
|-----------|-------|
| Rated voltage | **24 V DC** |
| Rated current | **5 A ± 10%** |
| Rated power | 84 W |
| Rated speed (motor shaft, no gearbox) | 3500 ± 10% RPM |
| Number of poles | 4 |
| Number of phases | 3 |
| Efficiency | 65% |
| Insulation class | Class B |
| Dielectric strength | 500 VAC for 1 minute |
| Insulation resistance | 100 MΩ min @ 500 VDC |
| Bearing type | NMB Ball Bearing, C3 |

> **Voltage note:** Rated at 24 V; the motive rail is 22.2 V nominal (25.2 V fully charged).
> Operating within this range is acceptable — the motor will run slightly under rated speed at
> 22.2 V but within safe operating limits. The BLD-510B controller handles voltage variation.

### INNEX-1 Power Budget

| Scenario | Current | Power |
|----------|---------|-------|
| Rated operation | 5 A | ~84 W |
| Peak (10% overload) | ~5.5 A | ~92 W |
| Controller fuse (BLD-510B slot) | **10 A** | Protects wiring against hard fault |

> Driving and excavation must **never run simultaneously** — this is a system-level rule enforced
> in software. See fuses.md and software-team-notes.md.

---

## Mechanical Specifications

| Parameter | Value |
|-----------|-------|
| Frame size | 57 mm (NEMA 23 equivalent), □57 mm body |
| Gear ratio | **100:1** (integrated gearbox) |
| Rated torque (output shaft) | **14.95 N·m** |
| Max permissible torque (output shaft) | **25 N·m** (221.27 lb-in) |
| Output shaft diameter | Ø14 mm |
| Keyway | 5 × 5 × 25 mm |
| Output speed (at rated conditions) | ~35 RPM (3500 RPM ÷ 100) |
| Backlash at no-load | 2.4° |
| Body length | 76 ± 1 mm |
| Mounting hole pattern | 4 × Ø5.2 mm on 47.14 ± 0.2 mm bolt circle |

---

## Hall Sensor & Phase Wiring

| Signal | Wire Colour | Lead Type |
|--------|------------|-----------|
| Hall A | Yellow | UL1007 26 AWG |
| Hall B | White | UL1007 26 AWG |
| Hall C | Blue | UL1007 26 AWG |
| Hall GND | Black | UL1007 26 AWG |
| Hall +5V | Red | UL1007 26 AWG |
| Phase U | Orange | UL1007 18 AWG |
| Phase V | Green | UL1007 18 AWG |
| Phase W | Brown | UL1007 18 AWG |

> **Hall sensor power:** +5V and GND connect to the BLD-510B controller's hall sensor supply
> pins — do not power from the Teensy or any other 3.3V rail.

---

## Motor Controller Interface

This motor is driven by the **BLD-510B BLDC motor controller**.

| Parameter | Value |
|-----------|-------|
| Controller | BLD-510B |
| Speed control | PWM from Teensy Pin 6 via 10 kΩ + 10 µF RC filter → SV pin |
| Direction | Teensy Pin 13 (GPIO, active-low F/R signal) |
| Enable | Teensy Pin 14 (GPIO, active-low EN signal) |
| Speed feedback | Teensy Pin 31 (PG pulse — open-collector, 10 kΩ pull-up to 3.3 V required) |
| Fault alarm | Teensy Pin 32 (ALM — open-collector, 10 kΩ pull-up to 3.3 V required) |
| Fuse (motive fuse block) | **10 A blade** (BLD-510B slot) |

> All control signals go through the Teensy 4.1. The Jetson sends excavation speed setpoints to
> the Teensy over USB serial; the Teensy translates these into BLD-510B control signals.

---

## Key Notes & Constraints

- **100:1 gearbox gives high torque at low speed** — output is ~35 RPM at rated conditions.
  This is appropriate for excavation duties but means back-driving is not possible.
- **Rated at 24 V, running at 22.2 V** — slight speed reduction at nominal voltage, no safety
  concern. Max voltage (25.2 V fully charged battery) is within acceptable range.
- **Hall sensor pull-ups:** PG and ALM on BLD-510B are open-collector — hardware 10 kΩ pull-up
  resistors to 3.3 V **must** be fitted. Without these, fault detection and speed feedback will
  not work.
- **Never run simultaneously with drivetrain** — combined peak draw would exceed 10 AWG motive
  trunk ampacity. Software interlock required.
- **Phase lead is 18 AWG** — match wiring from motor to BLD-510B at 18 AWG or heavier.
  Hall sensor leads are 26 AWG signal wires only.
- **Backlash 2.4°** — not significant for excavation use but worth noting for any positioning
  accuracy requirements.

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-24 | eniomecaj | Initial datasheet — sourced from 57BLR50-24-01-HG100 technical drawing (Aug 2023) |
