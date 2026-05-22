# Luxonis OAK-D Pro (DM9098) — Datasheet

## Overview

| Field | Value |
|-------|-------|
| **Manufacturer** | Luxonis |
| **Part Number** | OAK-D-PRO-AF (Auto Focus) / OAK-D-PRO-FF (Fixed Focus) |
| **Role on Rover** | Vision subsystem — RGB-D perception, AprilTag localisation, crater detection feeding Nav2 costmaps |
| **Power Domain** | 🔵 Compute (5 V via USB-C from Jetson Orin Nano) |
| **Qty on Rover** | 1 |
| **Official Datasheet** | [OAK-D-Pro_Datasheet.pdf](https://github.com/luxonis/oak-hardware/blob/master/DM9098_OAK-D-Pro/Datasheet/OAK-D-Pro_Datasheet.pdf) |
| **Luxonis Docs** | https://docs.luxonis.com/hardware/products/OAK-D%20Pro |

---

## Electrical Specifications

### Absolute Maximum Ratings

| Parameter | Min | Max | Unit |
|-----------|-----|-----|------|
| VBUS input voltage | 3.5 | 5.5 | V |
| Max input current | — | 2.0 | A |
| Ambient temperature | 0 | 60 | °C |

### Recommended Operating Conditions

| Parameter | Min | Typical | Max | Unit |
|-----------|-----|---------|-----|------|
| VBUS supply voltage | — | 5.0 | 5.25 | V |
| Power consumption (active, Pro features on) | 6 | — | **15** | W |
| Power consumption (idle, Myriad X booted) | — | 2.5 | — | W |
| Power consumption (base + camera streaming) | 2.5 | — | 3.0 | W |
| AI subsystem adder | — | — | +1.0 | W |
| Stereo depth pipeline adder | — | — | +0.5 | W |
| Video encoder adder | — | — | +0.5 | W |
| IR dot projector adder (configurable) | — | — | +1.0 | W |
| IR flood LED adder (configurable) | — | — | +1.0 | W |
| Ambient operating temperature | -20 | — | 50 | °C |

> **Peak draw is up to 15 W** when both IR projector and flood LED are running at full brightness
> alongside full VPU utilisation. The Jetson USB-C port is rated for 900 mA (4.5 W) under USB 2
> spec. **A Y-adapter is mandatory when using IR features** — see section below.

---

## Y-Adapter Power Requirement

When the IR dot projector or flood LED are enabled, the OAK-D Pro can exceed the 900 mA USB 2
current limit. The **Y-adapter** splits data (USB-C to host) and power (USB-C or barrel from a
separate supply) into a single USB-C connector on the camera.

```
Jetson USB-C ──── Y-adapter DATA leg  ──┐
                                         ├── OAK-D Pro USB-C
5V / 3A auxiliary PSU ── Y-adapter PWR ──┘
```

- The auxiliary power leg must supply **5 V at ≥ 3 A** (15 W headroom).
- Source this from the 14.8 V compute rail via a dedicated 5 V / 3 A DC-DC converter — do
  **not** tap the Jetson's USB power rail for this.
- Y-adapter part: Luxonis USB-C Y-adapter (sold separately) or equivalent USB-C PD splitter
  cable rated for 3 A.
- If running the rover without IR features (passive stereo + RGB only), the Y-adapter can be
  omitted and the camera runs on the Jetson USB port alone at ~3 W.

---

## Cable Sizing

| Connection | Cable Type | Recommended Rating | Notes |
|------------|-----------|-------------------|-------|
| Data (Jetson → OAK-D Pro) | USB-C 3.1 Gen 2 | ≥ 1 A, 10 Gbps rated | Keep ≤ 1 m for USB 3 reliability |
| Y-adapter aux power | USB-C or barrel | ≥ 3 A / 5 V | From dedicated 5 V DC-DC on compute rail |
| 5 V rail feed to DC-DC | 22–24 AWG | ≥ 3 A capacity | Fuse at 5 A on this branch |

---

## Camera Sensor Specifications

| Parameter | Color Camera (centre) | Stereo Cameras (×2) |
|-----------|----------------------|---------------------|
| Sensor | Sony IMX378 | OmniVision OV9282 |
| Resolution | 12 MP (4056 × 3040) | 1 MP (1280 × 800) |
| Max Framerate | 60 FPS | 120 FPS |
| Shutter | Rolling | **Global** |
| FOV (D/H/V) | 78° / 66° / 54° | 89.5° / 80° / 55° |
| Focus (AF variant) | Auto Focus: 8 cm – ∞ | Fixed Focus: 19.6 cm – ∞ |
| Focus (FF variant) | Fixed Focus: 50 cm – ∞ | Fixed Focus: 19.6 cm – ∞ |
| Pixel Size | 1.55 µm | 3 µm |
| IR Sensitive | No | **Yes** (940 nm notch filter) |

---

## IR Features

### IR Laser Dot Projector (Active Stereo)

| Parameter | Value |
|-----------|-------|
| Module | Ams Belago 1.1 VCSEL |
| Wavelength | 940 nm |
| Number of dots | 4700 |
| HFOI (50%) | 78° ± 7° |
| VFOI (50%) | 61° ± 7° |
| Max drive current | 1200 mA (≈ 1 W) |
| Operating temperature | 10 °C to ~60 °C |
| Absolute temperature limits | 0 °C to ~80 °C |
| Laser safety class | **Class 1** (EN/IEC 60825-1 Ed.3, 2014) |
| Default state | **OFF** — must be explicitly enabled via API |

> **Cold environment note:** At temperatures below 10 °C, allow 2–3 minutes of VPU warmup
> (running AI or stereo depth) before enabling the dot projector. The lunar analogue test
> environment may be cold — plan for this warmup in the mission startup sequence.

### IR Flood Illumination LED

| Parameter | Value |
|-----------|-------|
| Component | OSRAM SFH 4725AS |
| Wavelength | 940 nm |
| FOI | 80° |
| Max drive current | 1500 mA (≈ 1 W) |
| Compliance | IEC 62471:2006 |
| Default state | **OFF** — must be explicitly enabled via API |

---

## On-Board Compute (RVC2 / Myriad X VPU)

| Feature | Value |
|---------|-------|
| VPU | Intel Movidius Myriad X |
| Total compute | 4 TOPS |
| AI subsystem | 1.4 TOPS (neural network inference) |
| Encoding | H.264, H.265, MJPEG — 4K/30 FPS or 1080P/60 FPS |
| Flash | 256 Mbit QSPI NOR |
| IMU | BNO086 9-axis (accel + gyro + magnetometer) |

---

## Stereo Depth Performance

| Parameter | Value |
|-----------|-------|
| Stereo baseline | 75 mm |
| Ideal depth range | ~80 cm – 12 m |
| MinZ (800P mode) | ~80 cm |
| MinZ (400P + extended disparity) | ~20 cm |
| MaxZ | ~35 m |
| Depth accuracy < 4 m | < 2% absolute error |
| Depth accuracy 4–7 m | < 4% absolute error |
| Depth accuracy 7–10 m | < 6% absolute error |

---

## Mechanical Information

| Parameter | Value |
|-----------|-------|
| Dimensions (W × D × H) | **97 mm × 22.9 mm × 29.5 mm** |
| Weight | **91 g** |
| Bottom mount | ¼-20 UNC tripod thread |
| Rear VESA mount | M4 × 2, **75 mm spacing** (VESA standard) |
| Enclosure | Aluminium alloy |

### Mounting Notes for INNEX1

- **Preferred mount:** Use the **rear VESA M4 holes (75 mm pitch)** for a rigid bracket to the
  rover chassis. This eliminates tripod thread rocking and gives a fixed, repeatable camera pose
  for the TF tree in `lunabot_description`.
- The camera face must have **unobstructed forward view** — no rover structure within the stereo
  FOV (80° horizontal). Check against the URDF model.
- The dot projector and flood LED are on the **front face** and must not be blocked or have
  reflective surfaces within ~20 cm.
- Allow **≥ 5 mm clearance** around the aluminium body for thermal convection — the Myriad X
  runs hot under full VPU load. Do not enclose in foam or insulating material.
- Vibration: The OAK-D Pro has passed EN 60068-2-6:2008 vibration testing (see
  [Luxonis vibration report](https://www.luxonis.com/assets/marketing/certificates/Vibration_Test_Report.pdf)).
  Mount with thread-locking compound on M4 screws to prevent loosening on rough terrain.
- The USB-C port is on the **rear face**. Route the cable so it does not pull on the connector
  under chassis flex — use a cable tie anchor within 50 mm of the connector.

---

## INNEX1 Software Integration

### ROS 2 Package

| Item | Value |
|------|-------|
| ROS 2 Package | `lunabot_perception` |
| Driver | `depthai-ros` (community ROS 2 wrapper) |
| USB device | `/dev/bus/usb/...` — identified by Luxonis VID `03e7` |
| Key topics published | `/oak/rgb/image_raw`, `/oak/stereo/depth`, `/oak/imu/data` |

### Driver Install

```bash
sudo apt install ros-humble-depthai-ros
# Minimum depthai library version: 2.15+
pip install depthai>=2.15 --break-system-packages
```

### IR Feature API (Python / DepthAI)

```python
import depthai as dai

with dai.Device(pipeline) as device:
    # Enable dot projector at 50% (0.0 – 1.0 scale)
    device.setIrLaserDotProjectorIntensity(0.5)
    # Enable flood LED — 0.0 to disable (default)
    device.setIrFloodLightIntensity(0.0)
```

> **Safety:** Both IR sources are OFF by default. The mission startup sequence in
> `lunabot_bringup` should explicitly set projector intensity after confirming no personnel
> are in front of the rover. Add a check in `mission_manager` before enabling IR.

### udev Rule (stable device path)

```bash
# /etc/udev/rules.d/99-innex1.rules
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666", SYMLINK+="oakdpro"
```

---

## Known Constraints & Gotchas

- **Y-adapter is not optional with IR on:** Running IR features without the Y-adapter will
  cause USB undervoltage, device resets, or host port current limiting. Wire the aux supply
  before competition.
- **Dot projector cold start:** Below 10 °C the projector is outside its operating window.
  The rover startup script should include a ~3 minute VPU warmup phase before enabling
  active stereo in cold test environments.
- **Rolling shutter on RGB:** The IMX378 uses a rolling shutter. Fast lateral motion will
  cause image skew. Use the global-shutter mono cameras (OV9282) for AprilTag detection at
  speed — not the RGB stream.
- **IR flood and colour camera:** The IR flood LED (940 nm) is invisible to the IMX378 RGB
  camera. Night-vision only works on the mono (OV9282) cameras, which are IR-sensitive.
  Do not attempt RGB-based detection in IR-illuminated scenes.
- **USB 3 cable length:** Keep USB-C data cable ≤ 1 m for reliable 10 Gbps link. Longer
  runs require an active USB 3 extension cable — not a passive extender.
- **Thermal:** The VPU will throttle and eventually shut down above 105 °C (internal junction).
  Ambient limit is 50 °C. The aluminium enclosure acts as a passive heatsink — do not cover it.

---

## Laser Safety Compliance

This device is certified **Class 1** under EN/IEC 60825-1 Edition 3 (2014). Class 1 is safe
under all reasonably foreseeable conditions. No additional PPE is required for normal operation.

**Do not:**
- Power on if external physical damage is observed.
- Open the enclosure near the IR projector.
- Use magnifying optics (loupes, lenses) directed at the projector.
- Flash non-release firmware to the camera module.

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-22 | KJdotIO | Initial datasheet — sourced from official Luxonis datasheet (Dec 2021) and docs.luxonis.com |
