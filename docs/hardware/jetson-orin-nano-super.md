# NVIDIA Jetson Orin Nano Super Developer Kit — Datasheet

## Overview

| Field | Value |
|-------|-------|
| **Manufacturer** | NVIDIA |
| **Product** | Jetson Orin Nano Super Developer Kit |
| **Carrier Board Spec** | SP-11324-001 v1.3 (December 2024) |
| **Role on Rover** | High-level autonomy — navigation, perception, mission management |
| **Power Domain** | 🔵 Compute (12 V from BD-01 → DC jack, 9–20 V range) |
| **Qty on Rover** | 1 |
| **Price** | £280.60 |
| **Supplier** | RS Components |

---

## Module Specifications

| Parameter | Value |
|-----------|-------|
| Processor | NVIDIA Orin (ARM Cortex-A78AE) |
| AI Performance | 40 TOPS |
| Memory | 8 GB 128-bit LPDDR5 (up to 68 GB/s) |
| Storage | Micro SD card (UHS-1) |
| Networking | 10/100/1000 BASE-T Ethernet |
| OS | Linux (JetPack SDK) |
| Developer Kit weight | 0.175 kg |

---

## Electrical Specifications

| Parameter | Value |
|-----------|-------|
| **DC jack input voltage** | **9 – 20 V DC** |
| **INNEX-1 supply** | **12 V from BD-01** (within range ✅) |
| Max input current (at 19 V) | 4.2 A (~80 W max) |
| Typical power (compute load) | 7 – 25 W depending on performance mode |
| Module supply (internal) | 5 V only — carrier board always supplies VDD_IN at 5V regardless of module type |
| Main 5V rail capacity | 7.8 A max |
| Main 3.3V rail capacity | 5.4 A max |

> **Important:** The developer kit carrier board has MODULE_ID pull-up removed. VDD_IN is always
> 5V regardless of module. Custom carrier boards can support 5V/19V — the dev kit does not.
> This does not affect INNEX-1 operation since we use the dev kit as-is.

---

## Carrier Board Dimensions

| Parameter | Value |
|-----------|-------|
| Carrier board (PCB) | 100 × 79 mm |
| Full developer kit | 103 × 90.5 × 34.77 mm |

---

## Connectors & INNEX-1 Usage

| Connector | Spec | INNEX-1 Use |
|-----------|------|-------------|
| DC Jack (J16) | 9–20V input | 12V from BD-01 compute rail |
| GbE RJ45 (J15) | Gigabit Ethernet | GL-A1300 router (→ OS1 LiDAR + telemetry) |
| USB 3.2 Type A ×4 (J6, J7) | 5V, 0.5A max per port | OAK-D Pro ×2, Teensy 4.1 ×1, spare ×1 |
| USB Type C (J5) | Recovery mode | Not used in operation |
| CSI Camera ×2 (J20, J21) | 22-pin flex, 2.5 Gbps | **Not used** (cameras via USB) |
| 40-pin expansion (J12) | 3.3V/5V GPIO, SPI, I2C, UART | Available — not currently allocated |
| M.2 Key E (J10) | PCIe Gen3 x1, USB 2.0 | Available for future use |
| Fan (J13) | 5V, 0.15A | Cooling fan if required |

### USB Power Budget (per port)

| Port | Max Current | INNEX-1 Device |
|------|-------------|----------------|
| USB 3.2 Type A #1 | 0.5 A @ 5V | OAK-D Pro Camera #1 |
| USB 3.2 Type A #2 | 0.5 A @ 5V | OAK-D Pro Camera #2 |
| USB 3.2 Type A #3 | 0.5 A @ 5V | Teensy 4.1 (serial comms + 5V power) |
| USB 3.2 Type A #4 | 0.5 A @ 5V | Spare |

> **USB current limit:** Each Type A port is limited to 0.5A from the Jetson USB hub. The
> OAK-D Pro cameras may occasionally request more. Monitor for USB power warnings during
> testing; if instability occurs, consider an externally powered USB hub.

---

## Connectivity Architecture (INNEX-1)

```
BD-01 (12V) ─────────────────────── DC Jack (J16)
                                           │
                              Jetson Orin Nano Super
                                           │
              ┌────────────────────────────┼───────────────────┐
              │                            │                   │
         GbE (J15)                USB Type A ×3          USB Type A ×1
              │                    │       │                   │
        GL-A1300 Router      Teensy 4.1  OAK-D Pro ×2      Spare
              │
        OS1 LiDAR (via router LAN)
```

---

## Software Role

The Jetson runs the **high-level autonomy stack only**:
- ROS 2 (Humble) on Ubuntu
- Navigation, localisation, mission management
- Perception (OAK-D Pro depth/RGB via `depthai-ros`)
- LiDAR (`ouster-ros`)
- Commands sent to Teensy 4.1 over USB serial (`/dev/ttyACM0` or similar)
- Telemetry streamed to ground station via GL-A1300 router

> **The Jetson does not talk directly to any motor controller.** All motor I/O goes through
> the Teensy 4.1. See software-team-notes.md and teensy-4.1-microcontroller.md.

---

## Key Rules & Gotchas

- **Always power up Jetson and Teensy before applying 22.2V to Sabertooth** — floating signal
  lines on Sabertooth power-on can be interpreted as drive commands.
- **DC jack accepts 9–20V** — BD-01 at 12V is correct. Do not exceed 20V.
- **USB Type A ports limited to 0.5A each** — verify cameras operate stably at this limit.
- **60s LiDAR boot delay** — startup sequence must wait before expecting OS1 point cloud data.
- **ESD sensitive** — always ground yourself before handling the carrier board or module.
- **Do not hot-plug** peripherals while powered — shut down first before plugging/unplugging
  cameras, M.2 cards, or expansion headers.
- **Micro SD is primary storage** — use a fast UHS-1 card; slow cards cause JetPack performance issues.

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-24 | eniomecaj | Initial datasheet — sourced from SP-11324-001 v1.3 carrier board spec (Dec 2024) |
