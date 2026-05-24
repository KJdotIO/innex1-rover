# DollaTek 300W CC CV Buck Converter — Datasheet

## Overview

| Field | Value |
|-------|-------|
| **Manufacturer** | DollaTek |
| **ASIN** | B07DJ47HGZ |
| **Role on Rover** | Step-down 22.2 V motive rail → 12 V for Cytron MDD10A #2 (250mm actuators) |
| **Power Domain** | 🔴 Motive (input) → regulated 12 V output to Cytron MDD10A #2 |
| **Qty on Rover** | 1 |
| **Price** | £5.99 |
| **Supplier** | Amazon (DollaTek) |

---

## Electrical Specifications

| Parameter | Value |
|-----------|-------|
| Input voltage | **DC 7 – 32 V** |
| Output voltage | **DC 0.8 – 28 V** (continuously adjustable via potentiometer) |
| **INNEX-1 output set to** | **12 V DC** |
| Rated output current (advertised) | 12 A MAX |
| **Real output current limit** | **~8 A** (XL4016E chip rated 8 A — see note) |
| Constant current range | 0.2 – 12 A (adjustable) |
| Maximum output power | ~300 W |
| Conversion efficiency | Up to ~95% |
| Output ripple | ~50 mV (24 V in, 12 V out, 5 A load) |
| Dropout voltage | 1 V |
| No-load current | ~20 mA typical |
| Load regulation | ±1% (CV mode) |
| Voltage regulation | ±1% |
| Operating temperature | −40 °C to +85 °C (industrial grade) |
| Dimensions | 65 × 47 × 23.5 mm |
| Reverse polarity protection | **No** |
| Output backflow protection | **No** |

> **⚠️ Real current limit is ~8 A, not 12 A.** The XL4016E driver chip is rated 8 A maximum.
> Reviews confirm practical limit of ~7.7 A. INNEX-1 max draw through this converter is
> 2 × 3 A = 6 A (both 250mm actuators at full load) — within safe limits. Do not exceed 7 A.
> Add a heatsink or small fan if sustained loads above 5 A are expected.

---

## INNEX-1 Wiring

| Connection | Detail |
|------------|--------|
| Input + | Motive rail 22.2 V (from fuse block) |
| Input − | Motive GND |
| Output + | Cytron MDD10A #2 VPWR |
| Output − | Cytron MDD10A #2 GND |
| Output voltage | Set to **12 V** via CV potentiometer (close to input port, turn clockwise to increase) |
| Current limit | Set to **≥ 6 A** via CC potentiometer (close to output port) — leave at max unless current limiting is intentionally required |

> Set output voltage before connecting load. Use a multimeter on the output terminals to confirm
> 12 V before plugging in the Cytron.

---

## Potentiometer Adjustment

| Potentiometer | Location | Function | Direction |
|---------------|----------|----------|-----------|
| CV (voltage) | Close to **input** port | Sets output voltage | Clockwise = increase |
| CC (current) | Close to **output** port | Sets current limit | Clockwise = increase |

---

## Key Rules & Gotchas

- **No reverse polarity protection** — wiring polarity must be correct. Reversed input will
  likely damage the module.
- **Real current limit ~8 A** — do not load above 7 A sustained. INNEX-1 max is 6 A (well within limit).
- **Set voltage before connecting load** — verify 12 V output with multimeter first.
- **Heatsink recommended** — at sustained loads above 5 A, the power transistor can exceed
  65 °C. Add a small heatsink or 40 mm fan for reliability during competition runs.
- **No output backflow protection** — do not connect to a supply with higher output voltage on
  the load side; this will damage the module.
- **Not waterproof** — PCB mount only; house inside the chassis enclosure.

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-24 | eniomecaj | Initial datasheet — sourced from Amazon product page (ASIN B07DJ47HGZ, DollaTek) |
