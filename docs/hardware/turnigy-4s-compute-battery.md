# Turnigy 5000mAh 4S 40C — Datasheet

## Overview

| Field | Value |
|-------|-------|
| **Manufacturer** | Turnigy (HobbyKing) |
| **Part Number** | Turnigy 5000mAh 4S 40C LiPo Pack w/XT90 |
| **Role on Rover** | Secondary compute battery — powers Jetson, LiDAR, cameras, router, and all sensors via the 14.8 V compute rail. Stays alive during E-Stop. |
| **Power Domain** | 🔵 Compute (14.8 V nominal) |
| **Qty on Rover** | 1 |
| **Purchase Link** | https://hobbyking.com/en_us/turnigy-5000mah-4s-40c-lipo-pack-w-xt90.html |

---

## Electrical Specifications

| Parameter | Value |
|-----------|-------|
| Capacity | 5000 mAh (5 Ah) |
| Cell configuration | 4S1P (4 cells in series) |
| Nominal voltage | 14.8 V |
| Fully charged voltage | 16.8 V (4.2 V/cell) |
| Minimum safe voltage | 13.6 V (3.4 V/cell) — buzzer alarm threshold |
| Discharge rate (constant) | 40C |
| Discharge rate (burst, 10 s) | 80C |
| Max continuous current | **200 A** (5 Ah × 40C) |
| Max burst current | 400 A (5 Ah × 80C) |
| Balance plug | JST-XH |
| Discharge plug | **XT90** |

### INNEX1 Power Budget vs Battery Capacity

| Load | Peak Current | Battery Safety Factor |
|------|--------------|-----------------------|
| Full compute stack (Jetson + LiDAR + cameras + router) | ~10 A | **20×** (200 A / 10 A) |

> The compute battery is massively over-specced for current delivery by design. The 20×
> safety factor guarantees **zero voltage sag** on the 14.8 V rail, completely isolating
> the Jetson and Ouster LiDAR from any transients on the motive domain.

### Runtime Estimate

| Profile | Avg Current | Usable Capacity (80%) | Runtime |
|---------|------------|----------------------|---------|
| Full compute stack | ~6 A | 4 Ah | **~40 min** |

---

## Mechanical Specifications

| Parameter | Value |
|-----------|-------|
| Dimensions (L × W × H) | **143 × 51 × 38 mm** |
| Weight | **550 g** |

### Mounting Notes

- Secure with mechanical straps alongside the 6S motive battery in the neutral zone
- Balance lead (JST-XH) must remain accessible for the LiPo buzzer alarm connection
- Significantly lighter than the 6S pack (550 g vs 1110 g) — factor into CG

---

## Safety & Monitoring

### Low-Voltage Alarm
Same passive buzzer setup as the motive battery — JST-XH balance lead connected to a
**1S–8S LiPo buzzer alarm**, triggering at **3.4 V/cell** (13.6 V pack).

### E-Stop Behaviour
This battery is on the **compute rail which is fully isolated from the E-Stop circuit**.
When the 125 A E-Stop is triggered, this battery and everything it powers (Jetson, router,
LiDAR) remain live. This is intentional — telemetry and video feeds must stay up during
a safety stop.

### Charging
- Standard LiPo balance charger
- Recommended charge rate: 1C (5 A)
- Store at 3.8 V/cell (15.2 V pack) if unused for more than 48 hours
- Never charge below 3.0 V/cell or above 4.2 V/cell

---

## Known Constraints & Gotchas

- **Buck converter input range:** The 14.8 V rail feeds two XL4015 buck converters (BD-01
  → 19 V for Jetson, BD-02 → 5 V for sensors). Verify both converters accept input down
  to 13.6 V (minimum cell voltage) without dropping output regulation.
- **Voltage sag non-issue:** At a 20× safety factor the pack will never sag meaningfully
  under compute load. Voltage at the buck converter input will be stable throughout the run.
- **Do not cross-connect rails:** The compute ground and motive ground must remain isolated.
  Only signal wires (PWM, UART) cross between domains.

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-22 | eniomecaj | Initial datasheet |
