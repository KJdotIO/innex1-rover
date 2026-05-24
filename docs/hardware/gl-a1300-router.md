# GL.iNet GL-A1300 (Slate Plus) Router — Datasheet

## Overview

| Field | Value |
|-------|-------|
| **Manufacturer** | GL Technologies (Hong Kong) Limited |
| **Model** | GL-A1300 (Slate Plus) |
| **Role on Rover** | On-board LAN hub — connects OS1 LiDAR and Jetson; Wi-Fi link to ground station |
| **Power Domain** | 🔵 Compute — BD-02 5V rail (USB Type-C, 5V/3A) |
| **Qty on Rover** | 1 |

---

## Hardware Specifications

| Parameter | Value |
|-----------|-------|
| **CPU** | Qualcomm IPQ4018, Quad-Core @ 717 MHz |
| **Memory** | DDR3L 256 MB |
| **Storage** | NOR Flash 4 MB + NAND Flash 128 MB |
| **Ethernet ports** | 1 × WAN + 2 × LAN (all 10/100/1000 Mbps Gigabit) |
| **Wi-Fi** | 802.11 a/b/g/n @ 2.4 GHz (400 Mbps) + 802.11 ac @ 5 GHz (867 Mbps) |
| **USB** | 1 × USB 3.0 Type-A |
| **Power input** | USB Type-C, **5 V / 3 A** (15 W max) |
| **Power consumption** | < 6.5 W typical |
| **Dimensions** | 118 × 84 × 33 mm |
| **Weight** | 181 g |
| **OS** | OpenWrt |
| **Operating temperature** | 0 – 40 °C |

---

## INNEX-1 Power Supply

| Parameter | Value |
|-----------|-------|
| Supply source | BD-02 5V rail (compute domain) |
| Connection | USB Type-C cable from BD-02 WAGO output → router power port |
| Current draw | ~1.3 A @ 5 V (< 6.5 W) |

> Power is provided directly from the BD-02 WAGO 2-in-6-out block — no USB hub or splitter.
> The router is one of the 6 direct outputs from BD-02.

---

## INNEX-1 Network Topology

```
Ground Station (Wi-Fi 2.4 GHz)
          │
    GL-A1300 Router (Wi-Fi AP)
          │
    ┌─────┴──────┐
    │ LAN port 1 │ LAN port 2 │
    │            │
  OS1 LiDAR   Jetson Orin Nano Super
  (static IP)  (static IP)
```

| Port | Device | Connection | IP |
|------|--------|------------|----|
| **LAN 1** | Ouster OS1-128 LiDAR | GbE Ethernet | Static (configured in OS1 sensor web UI) |
| **LAN 2** | Jetson Orin Nano Super (J15) | GbE Ethernet | Static (configured in Ubuntu) |
| **WAN** | Not used | — | — |
| **Wi-Fi 2.4 GHz** | Ground station laptop/tablet | SSID set on router | DHCP or static |

> **2.4 GHz only for INNEX-1.** The ground station connects on 2.4 GHz — do not change to
> 5 GHz. 2.4 GHz provides better range and wall penetration for the competition arena.

---

## Network Configuration (INNEX-1 Setup)

### Router LAN Subnet
Set LAN subnet to e.g. `192.168.8.0/24` (GL.iNet default). Assign static IPs:

| Device | Recommended Static IP | Notes |
|--------|-----------------------|-------|
| Router LAN gateway | 192.168.8.1 | GL.iNet default |
| OS1-128 LiDAR | 192.168.8.50 | Configured via OS1 sensor web UI |
| Jetson Orin Nano | 192.168.8.100 | Configured in Ubuntu netplan/nmcli |
| Ground station | 192.168.8.200 | DHCP reservation or static |

> Static IPs prevent DHCP lookup delays during startup. The OS1 LiDAR requires a static IP to
> be reliably discovered by `ouster-ros`. See `ouster-os1-128-lidar.md` for OS1 config details.

### Wi-Fi SSID
Set a unique SSID (e.g. `INNEX-1-GCS`) with a strong password. Use 2.4 GHz band, channel
auto-select or fixed to a clear channel at the competition venue.

---

## Key Rules & Gotchas

- **Boot time ~30–60 s** — router must be given time to fully boot before the Jetson or OS1
  attempts network connections. Include router boot in the startup sequence.
- **OS1 LiDAR also has ~60 s boot delay** — total time from power-on to first point cloud data
  is at least 60 s even after the router is ready.
- **5 V/3 A required** — BD-02 must supply at least 3 A on the router output. Verify WAGO wire
  gauge and BD-02 converter rating support this.
- **Jetson Ethernet is the only LAN device feeding ROS** — the Jetson uses the router as its
  default gateway and LAN switch. All ROS topics traverse the router.
- **WAN port left disconnected** — no internet connection in the rover. The WAN port can be
  left empty. Do not plug LiDAR or Jetson into the WAN port by mistake.
- **OpenWrt admin UI** — accessible at 192.168.8.1 from any LAN or Wi-Fi connected device.
  Use this to configure static IPs, SSID, and DHCP. Change default admin password before
  competition.
- **Reset button** — press >3 s (< 8 s) to restore network settings; press ≥ 8 s to full factory
  reset. Do not accidentally hold reset during competition.
- **USB 3.0 port** — available for future use (e.g. USB storage or USB modem). Not used in
  current INNEX-1 design.

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-24 | eniomecaj | Initial datasheet — sourced from GL-A1300 Datasheet ver.20230602 (GL Technologies HK) |
