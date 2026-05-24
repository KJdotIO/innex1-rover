# Turnigy Graphene Professional 8000mAh 6S — Datasheet

## Overview

| Field | Value |
|-------|-------|
| **Manufacturer** | Turnigy (HobbyKing) |
| **Part Number** | Turnigy Graphene Professional 8000mAh 6S 15C w/XT90 |
| **Role on Rover** | Main motive power battery — feeds all drivetrain motors, excavation motor, and actuators via the 24 V motive rail |
| **Power Domain** | 🔴 Motive (22.2 V nominal) |
| **Qty on Rover** | 1 |
| **Purchase Link** | https://hobbyking.com/en_us/turnigy-graphene-professional-8000mah-6s-15c-lipo-pack-w-xt90.html |

---

## Electrical Specifications

| Parameter | Value |
|-----------|-------|
| Capacity | 8000 mAh (8 Ah) |
| Cell configuration | 6S1P (6 cells in series) |
| Nominal voltage | 22.2 V |
| Fully charged voltage | 25.2 V (4.2 V/cell) |
| Minimum safe voltage | 20.4 V (3.4 V/cell) — buzzer alarm threshold |
| Discharge rate (constant) | 15C |
| Discharge rate (burst) | 30C |
| Max continuous current | **120 A** (8 Ah × 15C) |
| Max burst current | 240 A (8 Ah × 30C) |
| Internal impedance | ≤ 1.2 mΩ/cell |
| Balance plug | JST-XH |
| Discharge plug | **XT90 anti-spark** |

### INNEX1 Power Budget vs Battery Capacity

| Scenario | Peak Current Draw | Battery Safety Factor |
|----------|------------------|-----------------------|
| Driving (4× drivetrain motors) | 40 A (fuse-capped) | **3.0×** (120 A / 40 A) |
| Excavation (motor + 4× actuators) | 45 A (fuse-capped) | **2.7×** (120 A / 45 A) |

> **Operational rule:** Driving and excavation must **never run simultaneously**. Peak demand
> is strictly capped at 45 A by local fuses. The battery will experience near-zero thermal
> stress or voltage sag under these conditions.

### Runtime Estimate

| Profile | Avg Current | Usable Capacity (80%) | Runtime |
|---------|------------|----------------------|---------|
| Driving | 15.2 A | 6.4 Ah | **~25 min** |
| Excavation | ~11.4 A | 6.4 Ah | ~34 min |

---

## Mechanical Specifications

| Parameter | Value |
|-----------|-------|
| Dimensions (L × W × H) | **170 × 69 × 48 mm** |
| Weight | **1110 g** (including wire, plug and case) |

### Mounting Notes

- Secure with mechanical straps — do not rely on friction alone given rover vibration
- Mount flat (largest face down) for best cell thermal distribution
- Keep clear of motor drivers in the enclosure — place in the **neutral zone** per the
  thermal layout (see `docs/hardware/electrical-box-layout.md`)
- Ensure balance lead (JST-XH) is accessible without removing the battery — needed
  for the LiPo buzzer alarm connection

---

## Safety & Monitoring

### Low-Voltage Alarm
A **1S–8S LiPo buzzer alarm** is connected directly to the JST-XH balance lead. It monitors
each cell independently and triggers a **95 dB audible alarm** if any cell drops below **3.4 V**.
This replaces an active BMS to avoid unwanted power cuts during high-load manoeuvres.

> No active BMS is used on this battery. The buzzer is the only low-voltage protection.
> Operators must respond to the alarm promptly and manually initiate a safe shutdown.

### Charging
- Use a standard LiPo balance charger (graphene chemistry is compatible with standard LiPo charge profiles)
- Recommended charge rate: 1C (8 A) for longevity; max fast charge: up to 15C on some cells — verify with charger
- Always charge in a LiPo-safe bag
- Never charge below 3.0 V/cell or above 4.2 V/cell

### Storage
- Store at 3.8 V/cell (22.8 V pack) if not in use for more than 48 hours
- Do not store fully charged or fully discharged

### Competition LiPo Rules
The passive buzzer alarm satisfies competition LiPo safety monitoring requirements by
providing real-time per-cell voltage audible telemetry without cutting motive power.

---

## Known Constraints & Gotchas

- **XT90 anti-spark connector:** The main battery uses an XT90 anti-spark variant. Ensure
  the mating connector on the harness is also XT90 anti-spark — a standard XT90 will work
  but loses the arc suppression on connection.
- **Voltage sag under load:** At 40 A draw, expect ~0.5–1 V sag from nominal 22.2 V.
  Motor controllers and drivers must tolerate input voltage down to ~20 V minimum.
  All selected drivers (Sabertooth 2×32, Cytron MD10C, BLD-510B) are rated down to this range.
- **Weight:** At 1110 g this is the heaviest single component in the electrical enclosure.
  Account for its position in the chassis CG calculation.
- **Graphene chemistry:** Charges and behaves like a standard LiPo — no special charger
  needed. The graphene structure primarily improves thermal performance and cycle life.

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-22 | eniomecaj | Initial datasheet |
