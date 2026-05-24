# INNEX-1 Fuse Strategy — Reference

## Overview

| Field | Value |
|-------|-------|
| **Fuse Block** | WUPP 12-way blade fuse block |
| **Fuse Block Rating** | 100 A total, 30 A max per slot |
| **Fuse Kit Included** | 5 A × 6, 10 A × 6, 15 A × 6, 20 A × 6 (standard blade) |

---

## Domain-Level Protection

Each power domain is protected by a single upstream fuse at the battery output. These are the primary protection for each rail — the fuse block and its branches rely on these.

| Domain | Fuse | Type | Location | Status |
|--------|------|------|----------|--------|
| 🔴 Motive (22.2 V) | **40 A** | Inline ANL | Between 6S battery and motive bus | ⚠️ Diagram shows 80 A — **must be updated to 40 A** |
| 🔵 Compute (14.8 V) | **20 A** | Inline blade | Between 4S battery and BD-01/BD-02 | ✅ Correct |

> **Why 40 A on the motive rail:** The main trunk is 10 AWG, rated ~35–40 A for short chassis
> runs. Driving and excavation never run simultaneously. With Sabertooth current limiting
> configured in software at 10 A per channel, worst-case driving draw is 4 × 10 A = 40 A and
> worst-case excavation is ~30 A. A 40 A ANL fuse sits at the wire's rated capacity and provides
> real protection. The theoretical gearbox-limit peak of 52 A never occurs because the Sabertooth
> caps it — making 40 A ANL the correct and safe choice for 10 AWG wiring.

> **⚠️ Diagram update required:** The wiring diagram (Wiring Diagram-PrimarySide) currently shows
> an 80 A ANL fuse on the motive domain. This must be corrected to **40 A**.

> **⚠️ Software dependency:** The 40 A ANL only works safely if Sabertooth current limiting is
> configured at startup (max 10 A per channel). If this is not set, the motive domain has no
> overcurrent protection below 40 A and the 10 AWG wiring is at risk under sustained peak load.
> See software-team-notes.md — this is a critical startup configuration.

---

## Fuse Block — Branch Fusing (Motive Rail)

The fuse block distributes the motive rail to individual controllers. The 60 A ANL upstream
is the domain protection. Branch fuses here provide per-device fault isolation.

| Slot | Device | Fuse | Reasoning |
|------|--------|------|-----------|
| 1 | Sabertooth 2×32 #1 (Left: FL + RL) | **No fuse** | 2 motors × 13 A = 26 A peak. 60 A ANL is domain protection. No suitable blade fuse in kit; adds complexity for minimal gain |
| 2 | Sabertooth 2×32 #2 (Right: FR + RR) | **No fuse** | Same as above |
| 3 | Cytron MDD10A #1 (Actuators 1 & 2) | **15 A** | 10 A per channel × 2 = 20 A max. 15 A protects against a single-channel fault without nuisance tripping |
| 4 | Cytron MDD10A #2 (Actuators 3 & 4) | **15 A** | Same as above |
| 5 | BLD-510B (Excavation BLDC) | **10 A** | Driver rated 10 A, motor rated 5 A continuous. 10 A fuse catches a hard fault before wiring damage |

**All fuses required are in the included kit — no additional orders needed for the fuse block.**

---

## What the Included Kit Fuses Are Used For

| Fuse Rating | Qty in Kit | Used | Where |
|-------------|-----------|------|-------|
| 5 A | 6 | 2 | Compute rail branches (LiDAR, cameras) via BD-02 |
| 10 A | 6 | 3 | Compute 20 A inline (separate), BLD-510B slot, BD-01 input |
| 15 A | 6 | 2 | Cytron MDD10A #1 and #2 slots |
| 20 A | 6 | 0 | Not required in current design |

---

## Key Rules

- Driving (drivetrain) and excavation must **never run simultaneously** — this is what keeps
  peak motive demand within the 60 A ANL rating
- The compute rail (20 A inline fuse) remains live through E-Stop by design — do not fuse it
  in a way that would cut it during a safety event
- Motive main trunk is 10 AWG — ANL fuse must be **40 A** to match wire ampacity. This is safe
  provided Sabertooth current limiting is enforced in software (10 A per channel max)

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-23 | eniomecaj | Initial fuse strategy document |
