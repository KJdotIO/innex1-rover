# DCHOUSE L11TGF1000N250 — Linear Actuator 250mm Datasheet

## Overview

| Field | Value |
|-------|-------|
| **Manufacturer** | DCHOUSE |
| **Part Number (MPN)** | L11TGF1000N250-T-1 |
| **ASIN** | B07S3VWKTB |
| **Role on Rover** | Linear actuation — deployment / adjustment mechanism |
| **Power Domain** | 🔴 Motive (via Cytron MDD10A — see voltage warning below) |
| **Qty on Rover** | 2 |
| **Price** | £35.99 (pair) |
| **Supplier** | Amazon (DCHOUSE GLOBAL-UK) |
| **Rating** | 4.3/5 — #1 Best Seller, Linear Motion Actuators |

---

## ⚠️ Voltage Warning

> **This actuator is rated 12V DC. The INNEX-1 motive rail is 22.2V nominal.**
>
> The Cytron MDD10A uses PWM switching — average voltage to the actuator depends on duty cycle.
> To stay within the actuator's 12V rating, **cap PWM duty cycle at approximately 55%**
> (55% × 22.2V ≈ 12.2V average). Running at 100% duty cycle from the 22.2V rail will over-voltage
> the actuator and risk burning out the motor winding.
>
> Software must enforce a duty cycle cap on all Cytron channels driving these actuators.
> See software-team-notes.md.

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
| **Red** | Motor + (extend) | Cytron MDD10A M1A (or M2A) |
| **Black** | Motor − (retract) | Cytron MDD10A M1B (or M2B) |

> **Direction:** Red to M1A / Black to M1B = actuator extends. Swap wires to reverse direction.
> If movement is wrong, swap at the Cytron terminals — do not invert in software first.

---

## Motor Controller Interface

| Parameter | Value |
|-----------|-------|
| Controller | Cytron MDD10A (dual channel) |
| Actuators per controller | 2 (one per channel) |
| Control type | **Open-loop — timed PWM commands, no position feedback** |
| Teensy PWM pins | Actuators 1 & 2: Pins 2, 3 / Actuators 3 & 4: Pins 4, 5 |
| Teensy DIR pins | Actuators 1 & 2: Pins 9, 10 / Actuators 3 & 4: Pins 11, 12 |
| **Max PWM duty cycle** | **~55%** to keep average voltage at 12V from 22.2V rail |
| Fuse (motive fuse block) | **15 A blade** per Cytron MDD10A slot |

---

## Key Rules & Gotchas

- **12V rated on a 22.2V rail — duty cycle must be capped at ~55% in software.** This is a
  critical constraint. Over-voltage will burn the actuator motor winding over time.
- **20% duty cycle** — maximum 1 minute on, 4 minutes off. Do not run continuously.
- **IP54** — dust and splash resistant but not fully sealed. Adequate for regolith environment
  provided the actuator is not directly submerged.
- **No encoder** — open-loop timed control only. Position cannot be verified; use timed commands
  with known extension rates (~10mm/s at rated load under correct voltage).
- **Inner limit switch is non-adjustable** — full stroke is always 250mm. No mid-stroke stopping
  via hardware; software must time out before end of travel if partial extension is needed.
- **Holds position when unpowered** — no need for active braking or holding current.

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-24 | eniomecaj | Initial datasheet — sourced from Amazon product page (ASIN B07S3VWKTB, DCHOUSE GLOBAL-UK) |
