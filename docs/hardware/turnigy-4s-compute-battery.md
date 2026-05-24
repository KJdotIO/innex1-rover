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
| BD-01 branch (Jetson ~4 A + LiDAR ~3 A) | ~6 A | — |
| BD-02 branch (cameras ~2 A + router ~1.3 A + IMU ~0.1 A) | ~6 A | — |
| **Full compute stack total** | **~12 A peak** | **16.7×** (200 A / 12 A) |

> The compute battery is massively over-specced for current delivery by design. The 16.7×
> safety factor guarantees **zero voltage sag** on the 14.8 V rail, completely isolating
> the Jetson and Ouster LiDAR from any transients on the motive domain.

### Runtime Estimate

| Profile | Avg Current | Usable Capacity (80%) | Runtime |
|---------|------------|----------------------|---------|
| Full compute stack (typical) | ~8 A | 4 Ah | **~30 min** |

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

## Key Notes & Constraints

- **Buck converter input range:** The 14.8 V rail feeds two buck converters: BD-01 (DollaTek
  Reg.NO:013726393, 14.8 V → 12 V for Jetson + LiDAR, ≤10 A) and BD-02 (Yosoo Health Gear
  g0qigxo64d, 14.8 V → 5 V for cameras/router/IMU, ≤10 A). Both must accept input down
  to 13.6 V (minimum cell voltage) without losing output regulation — verify during bench test.
- **Voltage sag non-issue:** At a 20× safety factor the pack will never sag meaningfully
  under compute load. Voltage at the buck converter input will be stable throughout the run.
- **Do not cross-connect rails:** The compute ground and motive ground must remain isolated.
  Only signal wires (PWM, UART) cross between domains.

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-22 | eniomecaj | Initial datasheet |
| 2026-05-24 | eniomecaj | Updated peak current to 12 A, SF to 16.7×, corrected BD-01/BD-02 names and voltages |
