# WUPP 12-Circuit Fuse Block — Datasheet

## Overview

| Field | Value |
|-------|-------|
| **Manufacturer** | WUPP |
| **ASIN** | B07GBST5NX |
| **Role on Rover** | Power Distribution Board — branch fusing for motive domain devices |
| **Power Domain** | 🔴 Motive (22.2 V input from 40A ANL fuse) |
| **Qty on Rover** | 1 |
| **Price** | £27.51 |
| **Supplier** | Amazon (KRS STORE LTD) |
| **Rating** | 4.7/5 (4,106 ratings) |

---

## Electrical Specifications

| Parameter | Value |
|-----------|-------|
| Input voltage | **DC 12 – 24 V** (compatible with 22.2 V motive rail) |
| Total block capacity | **100 A** |
| Max current per slot | **30 A** |
| Number of circuits | **12** (standard blade fuse, ATO/ATC) |
| Negative bus | **Yes** — integrated ground bus bar |
| LED fault indicator | **Yes** — red LED per circuit lights when fuse is blown |
| Connector type | Screw terminal |
| Mounting type | Chassis mount |
| Terminal material | Nickel-plated copper |
| Cover material | PBT (flame resistant, heat resistant, waterproof) |
| Cover type | Damp-proof removable cover |

---

## Included Fuse Kit

| Rating | Qty |
|--------|-----|
| 5 A | 6 |
| 10 A | 6 |
| 15 A | 6 |
| 20 A | 6 |

**All fuses required for INNEX-1 are in the included kit — no additional orders needed.**

---

## INNEX-1 Slot Assignments

For full fuse rationale see **fuses.md**.

| Slot | Device | Fuse Fitted |
|------|--------|-------------|
| 1 | Sabertooth 2×32 #1 (Left: FL + RL) | None — 40A ANL is domain protection |
| 2 | Sabertooth 2×32 #2 (Right: FR + RR) | None — 40A ANL is domain protection |
| 3 | Cytron MDD10A #1 (50mm Actuators 1 & 2) | **15 A** |
| 4 | Cytron MDD10A #2 input (→ DollaTek 12V buck → 250mm Actuators 3 & 4) | **15 A** |
| 5 | BLD-510B (Excavation BLDC) | **10 A** |
| 6–12 | Spare | — |

> **Why no fuse on Sabertooth slots:** The 40A ANL inline fuse between the battery and fuse block
> is the domain protection for the motive rail. The Sabertooth 2×32 draws up to 26A peak
> (2 motors × 13A) — no standard blade fuse in the included kit is appropriate without
> nuisance tripping. See fuses.md for full reasoning.

---

## Physical & Wiring Notes

- **Input:** Single feed from motive domain — connect after the 40A ANL inline fuse
- **Negative bus:** Connect all device GNDs to the negative bus bar for a clean star topology
- **LED indicators:** One red LED per slot — useful for fault diagnosis during testing and
  competition. A blown fuse causes the corresponding LED to illuminate
- **Cover:** Damp-proof cover should be fitted during operation to protect against regolith dust
- **Chassis mount:** Secure inside chassis enclosure with M-series bolts; keep accessible for
  fuse replacement during competition

---

## Key Rules & Gotchas

- **100A block total, 30A max per slot** — motive domain ANL is 40A so total block load is
  already capped well below the 100A block rating
- **Standard blade fuses only (ATO/ATC)** — do not fit mini or micro blade fuses; they will
  not make contact with the terminals correctly
- **LED on = blown fuse** — not a power indicator. If LED is on, check and replace the fuse
  before re-energising that branch
- **Damp-proof, not waterproof** — rated for damp environments, not submersion. Suitable for
  enclosed chassis with potential dust ingress but do not expose to water directly

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-24 | eniomecaj | Initial datasheet — sourced from Amazon product page (ASIN B07GBST5NX, WUPP) |
