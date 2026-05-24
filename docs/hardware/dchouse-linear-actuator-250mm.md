# DCHOUSE L11TGF1000N250 — Linear Actuator 250mm Datasheet

## Overview

| Field | Value |
|-------|-------|
| **Manufacturer** | DCHOUSE |
| **Part Number (MPN)** | L11TGF1000N250-T-1 |
| **ASIN** | B07S3VWKTB |
| **Role on Rover** | Linear actuation — deployment / adjustment mechanism |
| **Power Domain** | 🔴 Motive → DollaTek 12V buck converter → Cytron MDD10A #2 → actuators |
| **Qty on Rover** | 2 |
| **Price** | £35.99 (pair) |
| **Supplier** | Amazon (DCHOUSE GLOBAL-UK) |
| **Rating** | 4.3/5 — #1 Best Seller, Linear Motion Actuators |
| **Weight** | ~350 g each / ~700 g total (estimated) |

> **Voltage note:** This actuator is rated 12V DC. The motive rail is 22.2V. A DollaTek 300W
> buck converter steps the motive rail down to 12V before Cytron MDD10A #2 — no duty cycle
> capping required. See dollatek-buck-converter-12v.md.

---

## Electrical Specifications

| Parameter | Value |
|-----------|-------|
| **Rated input voltage** | **12 V DC** |
| No-load current | **0.8 A** |
| Max load current | **3 A** |
| Max push load | 1000 N (100 kg / 225 lb) |
| Max pull load | 1000 N (100 kg / 225 lb) |
| Travel speed (under load) | 10 mm/s |
| **Duty cycle** | **20% — must not run continuously** |
| Protection class | IP54 |
| Operating temperature | −26 °C to +65 °C |
| Protections built-in | Overcurrent, overload, overvoltage, short circuit, end-of-travel |

---

## Mechanical Specifications

| Parameter | Value |
|-----------|-------|
| Stroke length | **250 mm (10 inch)** |
| Retracted length | 370 mm |
| Extended length | 620 mm |
| Body material | Aluminium alloy |
| Shaft material | Alloy steel with metal gears |
| Limit switch | Inner — pre-installed, non-adjustable |
| Colour | Silver grey |

> The inner limit switch automatically stops the actuator at both ends of travel.
> The actuator holds position when unpowered.

---

## Wire Colours & INNEX-1 Connections

This actuator has **2 wires only** — no encoder.

| Wire Colour | Function | INNEX-1 Connection |
|-------------|----------|-------------------|
| **Red** | Motor + (extend) | Cytron MDD10A #2 M1A (or M2A) |
| **Black** | Motor − (retract) | Cytron MDD10A #2 M1B (or M2B) |

> **Direction:** Red to M1A / Black to M1B = actuator extends. Swap wires to reverse direction.
> If movement is wrong, swap at the Cytron terminals — do not invert in software first.

---

## Motor Controller Interface

| Parameter | Value |
|-----------|-------|
| Controller | Cytron MDD10A #2 (dual channel) |
| Supply to Cytron | 12 V from DollaTek buck converter (not direct motive rail) |
| Actuators per controller | 2 (one per channel) |
| Control type | **Open-loop — timed PWM commands, no position feedback** |
| Teensy PWM pins | Pins 4, 5 (Actuators 3 & 4) |
| Teensy DIR pins | Pins 11, 12 (Actuators 3 & 4) |
| Fuse (motive fuse block) | **15 A blade** — Cytron #2 slot (fuses the buck converter input) |

---

## Key Rules & Notes

- **Powered via 12V buck — no duty cycle restriction needed.** Cytron MDD10A #2 runs at 12V
  from the buck output; full PWM range is available.
- **20% duty cycle** — maximum 1 minute on, 4 minutes off. Do not run continuously.
- **IP54** — dust and splash resistant but not fully sealed. Adequate for regolith environment
  provided the actuator is not directly submerged.
- **No encoder** — open-loop timed control only. Use timed commands with known extension rate
  (~10mm/s at rated load).
- **Inner limit switch is non-adjustable** — full stroke is 250mm. Software must time out before
  end of travel if partial extension is needed.
- **Holds position when unpowered** — no active braking or holding current required.

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-24 | eniomecaj | Initial datasheet — sourced from Amazon product page (ASIN B07S3VWKTB, DCHOUSE GLOBAL-UK) |
| 2026-05-24 | eniomecaj | Removed PWM duty cycle warning — DollaTek 12V buck converter added before Cytron MDD10A #2 |
