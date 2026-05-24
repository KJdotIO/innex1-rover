# INNEX-1 Electrical Box Layout

Physical layout, zoning, component weights, and mounting strategy for the INNEX-1
electrical enclosure. Cross-reference with CDR Section 4 (Physical Integration) and
the mechanical team's CAD assembly.

---

## Enclosure Overview

| Parameter | Value |
|-----------|-------|
| Baseplate material | 5 mm polycarbonate |
| Cooling strategy | Single-direction active airflow — intake → compute zone → battery zone → motor driver zone → exhaust |
| Mounting clearance | ≥ 10 mm standoffs under all PCBs (airflow + solder joint protection) |
| Regolith protection | Filtered intake vents; motor bodies shielded within chassis |

---

## Zone Layout

```
AIR IN ──► [ COOL ZONE ]──►[ NEUTRAL ZONE ]──►[ HOT ZONE ] ──► AIR OUT
            Jetson, router    6S + 4S batteries   Sabertooth ×2
            cameras, Teensy   (strapped to base)   Cytron ×2, BLD-510B
            LiDAR, sensors                         WUPP fuse block
```

### Cool Zone (Intake Side)
Receives the coolest incoming air. Houses all heat-sensitive electronics.

| Component | Mounting | Standoff Height |
|-----------|----------|----------------|
| Jetson Orin Nano Super (with carrier board) | M3 through baseplate | 15–20 mm |
| GL-A1300 Router | Velcro / M3 screws | 10 mm |
| OAK-D Pro Cameras ×2 | Bracket to chassis wall (VESA M4 75 mm) | External — not in box |
| Teensy 4.1 | Breakout board, M2.5 screws | 10 mm |
| BD-01 Buck (DollaTek, 12V) | M2.5 screws / adhesive pad | 5 mm |
| BD-02 Buck (Yosoo, 5V) | M2.5 screws / adhesive pad | 5 mm |
| WAGO 2-in-4-out (BD-01 dist.) | Clip-mount to DIN rail or baseplate | — |
| WAGO 2-in-6-out (BD-02 dist.) | Clip-mount to DIN rail or baseplate | — |
| Inline fuse holder (20 A compute) | Cable-mount, zip-tied | — |
| Ouster OS1-128 LiDAR | Top-mounted bracket (external, above enclosure) | External |

### Neutral Zone (Centre)
Batteries are thermally isolated between the two active zones.

| Component | Mounting | Notes |
|-----------|----------|-------|
| Turnigy 6S 8000 mAh (motive) | Mechanical straps + foam padding to baseplate | Heaviest item — mount low and central for CG |
| Turnigy 4S 5000 mAh (compute) | Mechanical straps alongside 6S | Balance lead accessible for buzzer alarm |
| LiPo buzzer alarms ×2 | Zip-tied to battery balance leads | 1S–8S buzzer per battery |

### Hot Zone (Exhaust Side)
Highest heat-generating components nearest exhaust vents.

| Component | Mounting | Standoff Height |
|-----------|----------|----------------|
| Sabertooth 2×32 #1 (Left drive) | M3 screws, heatsink facing airflow | 12–15 mm |
| Sabertooth 2×32 #2 (Right drive) | M3 screws, heatsink facing airflow | 12–15 mm |
| Cytron MDD10A #1 (50 mm actuators) | M3 screws | 10 mm |
| Cytron MDD10A #2 (250 mm actuators, 12V) | M3 screws | 10 mm |
| BLD-510B BLDC Controller (excavation) | M3 screws, heatsink fins vertical | 12 mm |
| WUPP Fuse Block (PDB) | M3 screws | 10 mm |
| DollaTek 300W Buck (motive → 12V for Cytron #2) | M3 screws, adjacent to Cytron #2 | 8 mm |

---

## Component Weights

All weights are from manufacturer datasheets or measured values. Used for CG estimation
and chassis load planning.

| Component | Qty | Unit Weight | Total Weight | Source |
|-----------|-----|------------|-------------|--------|
| Turnigy 6S 8000 mAh LiPo | 1 | 1110 g | **1110 g** | Datasheet |
| Turnigy 4S 5000 mAh LiPo | 1 | 550 g | **550 g** | Datasheet |
| Jetson Orin Nano Super (module + carrier) | 1 | 175 g | **175 g** | Datasheet |
| Ouster OS1-128 LiDAR | 1 | ~370 g | **~370 g** | Ouster spec |
| Sabertooth 2×32 | 2 | 125 g | **250 g** | Datasheet |
| GL-A1300 Router | 1 | 181 g | **181 g** | Datasheet |
| OAK-D Pro Camera | 2 | 91 g | **~182 g** | Datasheet |
| Cytron MDD10A | 2 | ~100 g | **~200 g** | Estimated (84.5×62 mm PCB) |
| BLD-510B BLDC Controller | 1 | ~200 g | **~200 g** | Estimated (118×75.5×34 mm) |
| WUPP Fuse Block (PDB) | 1 | ~150 g | **~150 g** | Estimated |
| DollaTek 300W Buck (motive→12V) | 1 | ~80 g | **~80 g** | Estimated (65×47×23.5 mm) |
| BD-01 Buck (DollaTek, compute→12V) | 1 | ~40 g | **~40 g** | Estimated (small module) |
| BD-02 Buck (Yosoo, compute→5V) | 1 | ~30 g | **~30 g** | Estimated (small module) |
| Teensy 4.1 | 1 | 15 g | **15 g** | Datasheet |
| TRU Components 50mm Actuator | 2 | ~200 g | **~400 g** | Estimated |
| DCHOUSE 250mm Actuator | 2 | ~350 g | **~700 g** | Estimated |
| GR-WM4-V3 Drivetrain Motor | 4 | ~900 g | **~3600 g** | Estimated |
| 57BLR50 Excavation Motor | 1 | ~800 g | **~800 g** | Estimated |
| Wiring & connectors (estimate) | — | — | **~400 g** | Estimated |
| **Electrical system total** | | | **~9433 g ≈ 9.4 kg** | |

> Weights marked "Estimated" should be replaced with measured values once components arrive.
> Actuator and motor weights are for the bodies only — not including mounting brackets.

---

## Weight Summary

| Scope | Components | Weight |
|-------|-----------|--------|
| **Electrical box (enclosure contents only)** | Batteries, Jetson, Sabertooth ×2, Cytron ×2, BLD-510B, WUPP PDB, buck converters ×3, Teensy, wiring | **~3.2 kg** |
| External electrical — perception | Ouster OS1 LiDAR, GL-A1300 Router, OAK-D Pro ×2 | ~733 g |
| External electrical — actuation | TRU 50mm actuators ×2, DCHOUSE 250mm actuators ×2, 57BLR50 excavation motor | ~1900 g |
| External electrical — drivetrain | GR-WM4-V3 motors ×4 | ~3600 g |
| **Total electrical system** | All of the above | **~9.4 kg** |

> Motor and actuator weights are estimated. Replace with measured values once all components arrive.
> The electrical box alone (~3.2 kg) is the load the box baseplate and mounting hardware must support.

---

## Centre of Gravity Notes

- The two LiPo batteries (1660 g combined) dominate the electrical system mass — mount as
  low as possible in the chassis and as close to the geometric centre as the mechanical design allows.
- The Jetson + LiDAR are the most vibration-sensitive components — keep them on the rigid
  baseplate, not cantilevered off brackets.
- The four actuators are mounted externally on the mechanism — their weight affects rover CG
  but not the electrical box CG.

---

## Cable Routing

| Cable Group | Gauge | Route |
|-------------|-------|-------|
| Motive trunk (XT90 → ANL fuse → WUPP) | 10 AWG | Along chassis wall, shortest possible path |
| Sabertooth power (WUPP → B+/B−) | 10 AWG | 10 AWG direct from fuse block, ≤ 300 mm |
| Cytron #1 power (WUPP → Cytron) | 10 AWG | From slot 3 of WUPP |
| Cytron #2 power (WUPP → DollaTek buck → Cytron) | 10 AWG in, 12 AWG out | Via DollaTek 12V buck |
| BLD-510B power (WUPP slot 5 → controller) | 16 AWG | Short run, fused at 10 A |
| Compute trunk (4S → 20A fuse → BD-01/BD-02) | 18 AWG | Keep away from motive wiring |
| BD-01 → WAGO → Jetson + LiDAR | 20 AWG | Barrel jacks, short runs in cool zone |
| BD-02 → WAGO → Router + cameras | 20–22 AWG | USB-C cables |
| Signal wiring (Teensy ↔ controllers) | 24–26 AWG | Twisted pairs; route away from 10 AWG motor wires |
| Encoder wiring (motors → Teensy) | 26 AWG | Twisted CH-A/CH-B pairs per motor |

---

## Safety & Assembly Notes

- **Power-up sequence:** Compute domain (4S) powers up first → Jetson boots → Teensy initialises
  and asserts EN=stop on BLD-510B → only then apply motive domain (6S).
- **E-Stop location:** Mount on the exterior of the chassis within easy reach of the operator,
  upstream of the 40 A ANL fuse on the motive positive rail. Minimum 40 mm mushroom head diameter.
- **LiPo buzzer alarms:** Route balance leads to the front of the enclosure so audible alarms
  are not muffled by the enclosure walls.
- **Thermal:** Allow ≥ 5 mm clearance around the Sabertooth heatsinks and BLD-510B heatsink.
  Do not block exhaust vents. Competition regolith dust — use filtered intake vents.
- **Serviceability:** Batteries should be removable without disturbing signal wiring. Use
  connectors (XT90) rather than soldered joints on battery tails to allow quick swap.

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-24 | eniomecaj | Initial file — layout, weights, and routing notes for INNEX-1 electrical enclosure |
