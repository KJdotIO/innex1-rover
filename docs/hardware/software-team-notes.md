# INNEX-1 Software Team Notes
*Electrical hardware constraints and configuration requirements*

---

## ⚠️ Critical — Read First

### Sabertooth 2×32 Current Limiting
The GR-WM4-V3 drivetrain motors have a **gearbox bearing limit of 13 A per motor**. The Sabertooth
2×32 is capable of delivering up to 32 A per channel and will **not** protect the gearbox on its own.
Current limiting **must be configured in software** on the Sabertooth at startup. Sustained current
above 13 A will silently wear out the gearbox bearings — there is no mechanical fuse or trip, it will
just fail over time.

> Configure per-channel current limiting to **≤ 10 A per channel** via the Sabertooth serial API
> at startup before any motion commands are issued.

> **⚠️ Fuse safety dependency — this is not optional:** The motive domain is protected by a
> **40 A ANL fuse on 10 AWG wiring**. That fuse only keeps the wiring safe if the Sabertooth
> per-channel limit is set to **≤ 10 A**. With the limit in place, worst-case driving draw is
> 4 × 10 A = 40 A, which is within 10 AWG ampacity and matched by the ANL fuse. Without the
> limit, peak draw can reach 4 × 13 A = 52 A sustained — exceeding wire ampacity. The 40 A ANL
> would eventually blow, but not before the wiring is at risk. **Setting this limit is a wiring
> protection requirement, not just a gearbox protection measure.** See fuses.md for full rationale.

### Power-On Sequence
Always bring up the Jetson and Teensy **before** applying 22.2 V to the Sabertooth motor
controllers. Floating signal lines during Sabertooth power-on can be interpreted as drive commands.
The startup script must enforce this order.

---

## System Architecture

Three-layer signal hierarchy:

1. **Jetson ↔ Teensy** — USB serial (virtual COM), bidirectional
   - Jetson sends: velocity targets, actuator positions, BLDC speed setpoints
   - Teensy replies: encoder counts, positions, fault flags
2. **Teensy → Motor controllers** — UART (Sabertooth), PWM+DIR (Cytron), PWM (BLD-510B)
3. **Sensors → Teensy** — quadrature encoders, BLDC fault/speed pulses

The Jetson handles **high-level autonomy only**. All low-level motor I/O goes through the Teensy 4.1.

---

## Teensy 4.1 — Full Pin Map

| Pin(s) | Signal | Direction | Device |
|--------|--------|-----------|--------|
| 1 | Serial1 TX | → | Sabertooth #1 (Left: FL + RL) |
| 29 | Serial7 TX | → | Sabertooth #2 (Right: FR + RR) — remapped from Pin 8 (dead on breakout board) |
| 2, 3 | PWM ch1 & ch2 | → | Cytron MDD10A #1 (Actuators 1 & 2) |
| 9, 28 | DIR ch1 & ch2 | → | Cytron MDD10A #1 — Pin 28 remapped from Pin 10 (dead) |
| 4, 5 | PWM ch1 & ch2 | → | Cytron MDD10A #2 (Actuators 3 & 4) |
| 11, 12 | DIR ch1 & ch2 | → | Cytron MDD10A #2 |
| 6 | PWM (SV) | → | BLD-510B — excavation motor speed |
| 13 | GPIO (F/R) | → | BLD-510B — direction |
| 14 | GPIO (EN) | → | BLD-510B — enable |
| 15, 16 | Quad A, B | ← | Encoder: Front-Left motor |
| 17, 18 | Quad A, B | ← | Encoder: Rear-Left motor |
| 19, 20 | Quad A, B | ← | Encoder: Front-Right motor |
| 21, 22 | Quad A, B | ← | Encoder: Rear-Right motor |
| 31 | PG pulse | ← | BLD-510B — BLDC speed pulse |
| 32 | ALM | ← | BLD-510B — fault alarm |
| USB | Serial ↕ | ↕ | Jetson Orin Nano |

**~25 pins used — ~30 spare on Teensy 4.1**

> ⚠️ **Dead pins on breakout board (cold solder / pad fault):** 7, 8, 10, 35, 36, 40.
> Do NOT use these pins in firmware. Pin 8 works on bare Teensy pins but is dead on the breakout board — bench tests by KJ used bare pins directly. For the actual build on the breakout board, Sabertooth #2 is remapped to Pin 29.

---

## Sabertooth 2×32 — Drivetrain Controllers

Two controllers used — skid-steer topology:
- **Sabertooth #1** — Left side (FL + RL motors), Teensy Pin 1 (Serial1 TX)
- **Sabertooth #2** — Right side (FR + RR motors), Teensy Pin 29 (Serial7 TX) — remapped from Pin 8 (dead on breakout board)

Motor channel assignment:

| Controller | Channel | Motor |
|------------|---------|-------|
| Sabertooth #1 left | M1 | Front-Left |
| Sabertooth #1 left | M2 | Rear-Left |
| Sabertooth #2 right | M1 | Front-Right |
| Sabertooth #2 right | M2 | Rear-Right |

> **Bench result (2026-05-27):** All four drivetrain motors tested via Teensy serial firmware using bare Teensy pins (before breakout board). Left/right commands moved correct sides, arc and pivot commands produced correct encoder direction results. See `docs/teensy_drivetrain_bringup.md` for wiring checklist and encoder results.

> **Common wiring fault:** Both Sabertooth S1 inputs wired to Pin 1 causes all four motors to follow the left command. Correct mapping: left S1 → Pin 1, right S1 → Pin 29 (breakout board build).

### DIP Switch Settings (both controllers)
| Switch | Position | Reason |
|--------|----------|--------|
| SW1 | OFF | — |
| SW2 | OFF | — |
| SW3 | OFF | PSU mode (not battery) |
| SW4 | ON | Packetised serial |
| SW5 | ON | Address 128 |
| SW6 | OFF (competition) / ON (bench) | OFF = hardware E-stop via A1/A2 enabled. ON = software stop only (used during bench tests). Must be OFF for competition. |

### Software Configuration at Startup
```cpp
// Must be called once at startup before any motion commands
setSaberRamping(20);         // Hardware ramping — smooth accel/decel on all stop commands
// ⚠️ REQUIRED: set per-channel current limit to 10 A max
// Enforces gearbox bearing limit (13 A) and keeps motive draw within 40 A ANL fuse / 10 AWG wiring rating
// setCurrentLimit(10);     // Replace with actual Sabertooth packetised serial command for current limit
```

- Protocol: packetised serial, 9600 baud, address 128
- TX only — Teensy talks, Sabertooth listens (no RX needed)
- ROS 2 package: `lunabot_drivetrain`
- Command topic: `/cmd_vel_safe` (`geometry_msgs/msg/Twist`)

---

## GR-WM4-V3 Drivetrain Motors — Software Constraints

- **Encoder resolution:** 720 pulses/rev at the output shaft (12 PPR × ~64:1 gear ratio, both channels)
- **Encoder power:** Teensy 3.3 V rail — signals swing 0–3.3 V, safe for direct Teensy GPIO
- **No level shifters** required — do not power encoders from 5 V
- **Duty cycle limit:** Maximum 40% at rated load — 60 s on / 90 s minimum off. Mission planning
  must account for rest periods during sustained high-torque operations
- **Self-locking gearbox:** Holds position when stopped — no active braking needed for drivetrain
- Swapping CH-A and CH-B only reverses count direction — correctable in software, not a wiring fault

---

## BLD-510B — BLDC Excavation Motor Controller

### Speed Control (SV pin)
The SV pin accepts **1–2 kHz PWM directly** from Teensy Pin 6. No external RC filter is needed — the BLD-510B driver handles PWM internally. Duty cycle maps linearly to motor speed (0% = stop, 100% ≈ full speed at 3.3 V logic).

### Control Signal Logic
- EN (Pin 14), F/R (Pin 13) — active-low. Pull high to disable, pull low to enable/activate
- BK (brake) — not connected. EN is sufficient for all stop commands including emergency stops
- ⚠️ **Never call `Serial3.begin()`** — Serial3 TX = Pin 32, which is used for ALM input. Calling Serial3 will conflict with excavation fault monitoring

### Feedback Signals (open-collector — connect both)
PG and ALM are open-collector outputs — they only pull LOW, never drive HIGH. Fit a **10 kΩ pull-up to Teensy 3.3 V** on each pin. The HIGH state will be 3.3 V, which is safe for Teensy GPIO. No level shifter needed.

| Signal | Pin | Function |
|--------|-----|----------|
| PG | 31 | Speed pulse output — use for RPM feedback from excavation motor |
| ALM | 32 | Fault alarm — goes LOW on overcurrent, overheat, stall, or hall sensor fault. Treat as excavation emergency stop. |

- GND must be shared between BLD-510B signal ground and Teensy GND (already the case)
- ALM must be monitored continuously — any LOW triggers immediate excavation motor stop and fault report to ROS
- PG available for RPM calculation when needed by software team

### RC Filter Note (Arduino conflict — not applicable to Teensy)
The previous test setup used an Arduino powered from both USB and the driver's +5 V simultaneously,
which caused a voltage conflict and triggered the red alarm LED. On the Teensy/Jetson setup this
conflict does not apply, but **never power the Teensy from both USB and an external 5 V rail simultaneously**.

---

## OAK-D Pro Cameras — Software Notes

- ROS 2 package: `lunabot_perception`, driver: `depthai-ros`
- USB device: identified by Luxonis VID `03e7`
- udev rule (add to `/etc/udev/rules.d/99-innex1.rules`):

```bash
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666", SYMLINK+="oakdpro"
```

- Driver install:
```bash
sudo apt install ros-humble-depthai-ros
pip install depthai>=2.15 --break-system-packages
```

### Key Topics
| Topic | Type |
|-------|------|
| `/oak/rgb/image_raw` | RGB image stream |
| `/oak/stereo/depth` | Depth map |
| `/oak/imu/data` | IMU data |

### Critical Constraints
- **IR dot projector and flood LED are OFF by default** — they must be explicitly enabled via the
  DepthAI API. The mission startup sequence should only enable IR after confirming no personnel
  are in front of the rover:

```python
device.setIrLaserDotProjectorIntensity(0.5)  # 0.0 = off (default)
device.setIrFloodLightIntensity(0.0)          # 0.0 = off (default)
```

- **RGB camera uses a rolling shutter (IMX378)** — fast lateral motion causes image skew. Use
  the **mono cameras (OV9282, global shutter)** for AprilTag detection at speed, not the RGB stream
- **IR flood only illuminates for mono cameras** — the OV9282 stereo cameras are IR-sensitive;
  the RGB camera is not. Do not attempt RGB-based detection in IR-illuminated scenes
- Keep USB-C data cable ≤ 1 m for reliable USB 3 link

---

## Ouster OS1 LiDAR — Software Notes

- **Connection:** OS1 → GL-A1300 router (Gigabit Ethernet), router → Jetson (Gigabit Ethernet).
  The Jetson has only one Ethernet port — the LiDAR does **not** connect directly to the Jetson.
  Static IP configuration required on the router's LAN for the OS1.
- Power: 12 V from BD-01 compute rail (~2–3 A)
- **Boot time: ~60 seconds** from power-on before the sensor begins spinning and publishing data.
  The mission startup sequence must include a 60 s wait after LiDAR power-on before expecting
  point cloud data
- Default operating mode: 1024 × 10 Hz on boot
- ROS 2 driver: `ouster-ros` (official Ouster package)

---

## GL-A1300 Router — Software Notes

- Must operate on **2.4 GHz band**
- Telemetry stream must remain **under 4,000 Kbps** to stay within bandwidth limits
- Connected to Jetson via Gigabit Ethernet (sole Ethernet port on Jetson)
- OS1 LiDAR also connects to this router — the router acts as the LAN hub for both Jetson and LiDAR
- Assign static IPs on the router's LAN for both the OS1 and the Jetson interface

---

## Actuators (Cytron MDD10A) — Software Notes

- Actuators run **open-loop** — timed commands, no encoder feedback to Teensy
- Cytron MDD10A library: install "Cytron Motor Drivers" via Arduino IDE Library Manager
  (or use direct PWM/DIR for Teensy implementation)
- Direction reversal: swap M1A/M1B wires — do not invert in software first
- Never connect 24 V directly to the Teensy/microcontroller side

---

## E-Stop Behaviour

- The 250 A E-Stop cuts **motive power only** (22.2 V rail)
- The compute rail (14.8 V) remains live through E-Stop — Jetson, LiDAR, router, and cameras
  stay up for telemetry and video during a safety stop
- Software must not assume the whole system is dead after E-Stop — autonomy stack keeps running

---

## Git / PR Workflow

- Branch naming: descriptive (e.g. `fix/oak-d-author`, `docs/lidar-datasheet`)
- Commit prefix convention: `Docs:` for documentation, align with team convention
- **CodeRabbit** auto-reviews every PR on the repo — expect automated comments
- Datasheets live in `docs/hardware/` inside the repo
- PRs require KJ's approval before merge — direct push to `main` is blocked

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-23 | eniomecaj | Initial hardware notes |
| 2026-05-23 | eniomecaj | Clarified Sabertooth current limit to ≤ 10 A/channel; added fuse safety dependency note; updated TODO to required action with wiring safety context |
| 2026-05-23 | eniomecaj | Fixed network topology: OS1 connects via router, not direct to Jetson (Jetson has one Ethernet port); corrected router band to 2.4 GHz |
| 2026-06-02 | eniomecaj | Pin remaps: Sabertooth #2 Pin 7→29 (UART7), Cytron #1 DIR ch2 Pin 10→28. Dead pins added. RC filter removed from SV pin. PG/ALM pull-ups made optional. E-stop updated to 250A. SW6 corrected to OFF |
| 2026-06-02 | eniomecaj | Sabertooth #1 corrected to Pin 1 (Serial1 TX). PG/ALM wiring finalised: 10kΩ to 3.3V via dual-bus WAGO. BK removed (not connected). Serial3/Pin32 conflict warning added. |
