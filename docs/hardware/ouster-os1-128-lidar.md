# Ouster OS1-128 LiDAR — Datasheet

## Overview

| Field | Value |
|-------|-------|
| **Manufacturer** | Ouster |
| **Part Number / Model** | OS1-128, Rev 7 |
| **Role on Rover** | Primary 3D perception — obstacle detection, localisation, mapping |
| **Power Domain** | 🔵 Compute (12 V from BD-01) |
| **Qty on Rover** | 1 |
| **Acquisition** | Sponsored — estimated value £16,809 |
| **Manual** | OS1 Hardware User Manual Rev 7 (Oct 14, 2024) |
| **Product Page** | https://ouster.com/products/hardware/os1 |
| **Weight** | ~370 g (Ouster OS1 spec) |

---

## Sensor Specifications

| Parameter | Value |
|-----------|-------|
| Beam count (vertical channels) | **128** |
| Beam spacing options | Uniform, Gradient, Above Horizon, Below Horizon |
| Horizontal resolution (default) | 1024 points/revolution |
| Rotation rate (default) | 10 Hz |
| Interface | Gigabit Ethernet (1000BASE-T, RJ45) |
| Mechanical dimensions | Identical across OS1-32 / OS1-64 / OS1-128 |
| IP rating | All-weather, indoor/outdoor rated |
| Operating temperature | Per OS1 Rev 7 spec |

> The OS1-128 provides the highest vertical resolution in the OS1 family. All three models (32/64/128)
> share identical mechanical dimensions — mounting hardware is common across variants.

---

## Electrical Specifications

| Parameter | Value |
|-----------|-------|
| Input voltage (nominal) | **12 V – 24 V DC** |
| Input voltage (minimum before warning) | 9.5 V (low voltage warning triggered) |
| Input voltage (shutdown threshold) | 9.0 V |
| Input voltage (maximum) | 34 V |
| Typical power consumption | ~20 W (~1.7 A at 12 V) |
| Peak power consumption (cold, −40 °C) | ~28 W (~2.3 A at 12 V) |
| Recommended minimum supply | **12 V, ≥ 3.3 A** (≥ 40 W supply capability) |
| Power supply type | LPS-rated, 12 V DC, 5.5 × 2.5 mm centre-positive barrel |

### INNEX-1 Power Source

The OS1-128 is powered from the **BD-01 buck converter** (12 V compute rail), which also supplies
the Jetson Orin Nano. Total compute rail draw: ~1.7–2.3 A (LiDAR) + ~3–4 A (Jetson) = **~5–6 A
combined**. BD-01 must be rated accordingly.

---

## Network / Connectivity

| Parameter | Value |
|-----------|-------|
| Interface | Gigabit Ethernet (1000BASE-T) |
| Connector on sensor cable | RJ45 (via Interface Box) |
| INNEX-1 topology | OS1 → GL-A1300 router (GbE) → Jetson Orin Nano (GbE) |
| IP configuration | Static IP required — assigned on GL-A1300 LAN |

> **Network topology note:** The Jetson has a single Ethernet port occupied by the router. The OS1
> connects to the router's LAN port, not directly to the Jetson. Both OS1 and Jetson require static
> IP addresses on the router's LAN.

---

## Cable Pinout (Type 3 — 1000BASE-T, 12 V)

This is the cable type relevant to INNEX-1 (12 V operation, standard Ethernet).

| Function | Pin | Wire Colour | Wire Gauge |
|----------|-----|-------------|------------|
| VCC (12 V) | 1 | Red | 18 AWG |
| Ground | 7 | Black | 18 AWG |
| MULTIPURPOSE_IO | 3 | Purple | 28 AWG |
| SYNC_PULSE_IN | 2 | Yellow | 28 AWG |
| Ethernet BI_DA+ | 5 | White/Orange | 28 AWG |
| Ethernet BI_DA- | 4 | Orange | 28 AWG |
| Ethernet BI_DB+ | 8 | White/Green | 28 AWG |
| Ethernet BI_DB- | 6 | Green | 28 AWG |
| Ethernet BI_DC+ | 9 | White/Blue | 28 AWG |
| Ethernet BI_DC- | 10 | Blue | 28 AWG |
| Ethernet BI_DD+ | 12 | White/Brown | 28 AWG |
| Ethernet BI_DD- | 11 | Brown | 28 AWG |

> A black shielding drain wire is included for EMI bonding — leave unconnected unless needed for
> EMC testing.

---

## Mechanical Interface

| Parameter | Value |
|-----------|-------|
| Included with sensor | Sensor, interface box cable/connector, 24 V power supply (2 m), RJ45 cable (1 m), baseplate mount |
| Mounting | Baseplate mount — bolt through flange |
| Orientation | Fixed upward-facing mount recommended for full 360° coverage |

> **INNEX-1 note:** We are powering direct from BD-01 at 12 V rather than using the supplied 24 V
> power supply. The High Current Interface Box supports 12 V operation — verify the included
> interface box type is the High Current variant (PCBA 830-104249 or PCBA PN-830-103125).

---

## Software Integration

| Item | Value |
|------|-------|
| ROS 2 driver | `ouster-ros` (official Ouster package) |
| Default mode on boot | 1024 × 10 Hz |
| **Boot time** | **~60 seconds** from power-on — sensor must spin up before point cloud data is published |
| Connection | Gigabit Ethernet, static IP via GL-A1300 router |
| Compute rail | 🔵 BD-01 12 V |

> **⚠️ Critical startup constraint:** The mission startup sequence must include a **60-second wait**
> after LiDAR power-on before expecting any point cloud data. Attempting to use `/ouster/points`
> before the sensor has completed its startup will result in no data. See software-team-notes.md.

---

## Key Rules & Notes

- **Boot delay is mandatory** — 60 s from power-on before data is available. Build this into the
  startup sequence explicitly; do not rely on topic availability as a proxy.
- **12 V supply at ≥ 3.3 A capability** — BD-01 must be able to deliver this alongside Jetson load.
  Total compute rail draw ~5–6 A; verify BD-01 rating covers this.
- **Static IP required** — the OS1 does not use DHCP by default. Assign a fixed IP on the
  GL-A1300 LAN for both the OS1 and the Jetson interface.
- **Not direct-connected to Jetson** — topology is OS1 → router → Jetson. All network
  configuration (IP, firewall, routing) goes on the GL-A1300.
- **Low voltage warning at 9.5 V, shutdown at 9.0 V** — ensure no voltage sag on the 12 V rail
  under peak combined load (LiDAR + Jetson). Buck converter must be stable.
- **Interface Box type matters** — the included 24 V power supply and Legacy Interface Box are
  not compatible with direct 12 V operation. Use the High Current Interface Box which supports
  12–24 V input.

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-24 | eniomecaj | Initial datasheet — sourced from OS1 Hardware User Manual Rev 7 (Oct 2024) |
