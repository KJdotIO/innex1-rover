# Teensy 4.1 — Datasheet

## Overview

| Field | Value |
|-------|-------|
| **Manufacturer** | PJRC |
| **Part Number** | TEENSY41-L (lock variant) |
| **MPN** | TEENSY41_LOCK |
| **Role on Rover** | Low-level motor I/O controller — bridges Jetson to all motor controllers and encoders |
| **Power Domain** | 🔵 Compute (5 V from BD-02) |
| **Qty on Rover** | 1 |
| **Supplier** | PiHut |
| **Price** | £30.30 |
| **Product Page** | https://www.pjrc.com/store/teensy41.html |

---

## Electrical Specifications

| Parameter | Value |
|-----------|-------|
| Processor | IMXRT1062DVJ6 ARM Cortex-M7 |
| Clock speed | 600 MHz |
| Flash memory | 7,936 kB |
| RAM | 1,024 kB |
| EEPROM | 1,080 B (emulated) |
| **Supply voltage** | **3.6 – 5.5 V** |
| **Logic level** | **3.3 V** |
| **Max current per IO pin** | **10 mA** |
| Total digital pins | 55 |
| Analog inputs | 18 |
| Ethernet | 10/100 Mbit (DP83825 PHY) |
| Weight | 15 g |

### Key Electrical Rules for INNEX-1

- All GPIO signals are **3.3 V logic** — do not connect 5 V signals directly to any pin
- Max **10 mA per IO pin** — do not drive loads directly; use buffers or pull-ups where needed
- Encoders are powered from the Teensy **3.3 V rail** — signals swing 0–3.3 V, safe for direct GPIO
- **Never power the Teensy from both USB and an external 5 V rail simultaneously** — voltage conflict
- Open-collector signals (BLD-510B PG and ALM) require **10 kΩ pull-up resistors to 3.3 V** on
  the PCB/breadboard — do not rely on internal pull-ups for these

---

## INNEX-1 Pin Assignment

| Pin(s) | Signal | Direction | Device | Notes |
|--------|--------|-----------|--------|-------|
| **0** | UART1 TX | → | Sabertooth 2×32 #1 | Left drivetrain (FL + RL). Packetised serial, 9600 baud, address 128 |
| **7** | UART2 TX | → | Sabertooth 2×32 #2 | Right drivetrain (FR + RR). Packetised serial, 9600 baud, address 128 |
| **2** | PWM ch1 | → | Cytron MDD10A #1 | Actuator 1 speed |
| **3** | PWM ch2 | → | Cytron MDD10A #1 | Actuator 2 speed |
| **9** | DIR ch1 | → | Cytron MDD10A #1 | Actuator 1 direction |
| **10** | DIR ch2 | → | Cytron MDD10A #1 | Actuator 2 direction |
| **4** | PWM ch1 | → | Cytron MDD10A #2 | Actuator 3 speed |
| **5** | PWM ch2 | → | Cytron MDD10A #2 | Actuator 4 speed |
| **11** | DIR ch1 | → | Cytron MDD10A #2 | Actuator 3 direction |
| **12** | DIR ch2 | → | Cytron MDD10A #2 | Actuator 4 direction |
| **6** | PWM (SV) | → | BLD-510B | Excavation speed — via 10 kΩ + 10 µF RC filter to SV pin |
| **13** | GPIO (F/R) | → | BLD-510B | Excavation direction — active-low |
| **14** | GPIO (EN) | → | BLD-510B | Excavation enable — active-low |
| **15** | Encoder A | ← | FL motor encoder | Front-Left quadrature CH-A |
| **16** | Encoder B | ← | FL motor encoder | Front-Left quadrature CH-B |
| **17** | Encoder A | ← | RL motor encoder | Rear-Left quadrature CH-A |
| **18** | Encoder B | ← | RL motor encoder | Rear-Left quadrature CH-B |
| **19** | Encoder A | ← | FR motor encoder | Front-Right quadrature CH-A |
| **20** | Encoder B | ← | FR motor encoder | Front-Right quadrature CH-B |
| **21** | Encoder A | ← | RR motor encoder | Rear-Right quadrature CH-A |
| **22** | Encoder B | ← | RR motor encoder | Rear-Right quadrature CH-B |
| **31** | PG pulse | ← | BLD-510B | BLDC speed pulse — 10 kΩ pull-up to 3.3 V required |
| **32** | ALM | ← | BLD-510B | BLDC fault alarm — 10 kΩ pull-up to 3.3 V required |
| **USB** | Serial ↕ | ↕ | Jetson Orin Nano | USB virtual COM — bidirectional command/telemetry |

**~25 pins used — ~30 pins spare on Teensy 4.1**

---

## Power Rails Provided by Teensy

| Rail | Used For | Notes |
|------|----------|-------|
| **3.3 V** | Encoder VCC (all 4 drivetrain motors) | Two 3.3 V pins feed a WAGO 2-in-4-out to supply all 4 encoder VCC wires |
| **GND** | Encoder GND, signal ground | Separate WAGO output from motor controller GND — star topology |

> Encoders must **not** be powered from BD-02 5 V or any other rail. The SS460S Hall sensors
> operate from 3 V minimum, so 3.3 V is correct and signals stay within Teensy GPIO safe range.

---

## Communication Interfaces Used

| Interface | Protocol | Connected To |
|-----------|----------|-------------|
| USB (virtual COM) | Serial | Jetson Orin Nano — bidirectional |
| UART1 (Pin 0) | Packetised serial, 9600 baud | Sabertooth 2×32 #1 (TX only) |
| UART2 (Pin 7) | Packetised serial, 9600 baud | Sabertooth 2×32 #2 (TX only) |
| GPIO PWM (Pins 2–6) | PWM + DIR | Cytron MDD10A #1 and #2, BLD-510B SV |
| GPIO (Pins 13–14) | Digital out, active-low | BLD-510B F/R and EN |
| GPIO (Pins 15–22) | Quadrature encoder input | 4× GR-WM4-V3 drivetrain motor encoders |
| GPIO (Pins 31–32) | Open-collector input | BLD-510B PG and ALM |

---

## Mounting & Physical

- **Weight:** 15 g
- Mount securely inside chassis — vibration from drivetrain and excavation can cause connector loosening
- Keep USB cable to Jetson short and strain-relieved
- **Lock variant (TEENSY41-L):** firmware can be permanently locked against readout — do not activate lock during development; only consider for final competition firmware if required

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-24 | eniomecaj | Initial datasheet — sourced from TEENSY41-L datasheet (joy-it.net, Feb 2025) and INNEX-1 pin allocation files |
