# INNEX-1 Connectors & Accessories Reference

Single reference for all connector, passive, and accessory items purchased. Not individual
component datasheets — just enough to know what each item is for and any key points to watch.

---

## Power Distribution & Regulation

### BD-01 — 12 V Buck Converter (DollaTek Reg.NO:013726393, £7.77, Amazon)
- **Role:** Steps 14.8 V compute battery down to 12 V for Jetson Orin Nano + OS1 LiDAR
- **Rated:** ≤ 10 A output confirmed (Jetson ~3–4 A + LiDAR ~2–3 A = ~6 A combined, 10 A headroom)
- **Output distribution:** 12 V output feeds WAGO 2-in-4-out block → Jetson (pigtail barrel jack) + LiDAR (pigtail barrel jack), 2 spare outputs

### BD-02 — 5 V Buck Converter (Yosoo Health Gear g0qigxo64d, £3.22, Switch Electronics)
- **Role:** Steps 14.8 V compute battery down to 5 V for OAK-D Pro cameras (×2) + GL-A1300 router + IMU
- **Rated:** ≤ 10 A output confirmed. Actual load: cameras ~2 A + router ~1.3 A + IMU ~0.1 A = ~3.4 A
- **Output distribution:** 5 V output feeds WAGO 2-in-6-out block (3 active outputs: router, camera ×2)

---

## WAGO Connectors

### WAGO 2-in-6-out (£5.53, Amazon)
- **Role:** BD-02 output distribution — 5 V to cameras ×2 and GL-A1300 router (3 active outputs, 3 spare)
- **Status:** ✅ This is the current design — replaces the 2-in-4-out block
- ⚠️ Ensure wire gauge matches WAGO clamp rating (check product page — typically rated for 0.5–6 mm² / 20–10 AWG)

### WAGO 2-in-4-out (£9.89, Amazon)
- **Role:** BD-01 12 V output distribution — Jetson (pigtail barrel jack) + LiDAR (pigtail barrel jack)
- **Status:** ✅ Active — 2 outputs in use (Jetson + LiDAR), 2 spare

---

## Power Connectors & Cabling

### XT90 Anti-Spark Connector (£7.49, eBay)
- **Role:** Main battery disconnect on the motive domain (6S LiPo)
- **Placement:** Between the 6S LiPo positive terminal and the 40 A ANL fuse holder
- ⚠️ Anti-spark only works in one orientation — the resistor pre-charge pin must be on the battery side (male plug on battery, female on harness). Verify orientation before soldering.
- ⚠️ Solder properly — XT90 carries full motive current. Use a temperature-controlled iron and heat the barrel, not the wire.

### 10 AWG Wire Black and Red (£14.99, Amazon)
- **Role:** Main battery cables — motive trunk from XT90 → ANL fuse → WUPP fuse block, and motor power runs to Sabertooth
- **Note:** 10 AWG is the spec confirmed in CDR (updated from 6 AWG). Use for all motive domain trunk wiring.

### Yellow Ring Terminal 8.4mm (£7.69, Amazon)
- **Role:** Battery terminal connections at the XT90 / ANL fuse ends
- **Note:** 8.4mm inner diameter fits M8 bolts. Confirm your battery terminal bolt size before crimping. Crimp with proper ratchet crimper — a bad crimp on the main battery lead is a fire risk.
- Yellow ring terminals are rated for 12–10 AWG — 10 AWG wire fits correctly ✅

### USB-C to 2-Pin Bare Wire (×2, £8.99, Amazon)
- **Role:** 5 V power feed cables from BD-02 WAGO → GL-A1300 router (USB-C) and any USB-C powered device
- **Note:** Verify polarity of bare wires before connecting to WAGO — red = +5 V, black = GND. USB-C can be damaged by reverse polarity.

### 6× 5.5mm × 2.5mm 90° DC Power Male Plug (£6.99, Amazon)
- **Role:** DC barrel jack connector for **Ouster OS1 LiDAR** 12 V power input (from BD-01 via WAGO 2-in-4-out)
- **Note:** OS1 LiDAR DC jack is **5.5mm OD × 2.5mm ID center-positive** ✅. The 90° angle helps with cable routing. Solder wire to plug before heatshrinking — centre pin = positive (12 V), outer shell = GND.

### Pigtail to DC Barrel Jack (£7.19, Amazon)
- **Role:** Pre-made pigtail cable for **Jetson Orin Nano** power input (J16 on carrier board, from BD-01 via WAGO 2-in-4-out)
- **Note:** Confirm 5.5mm × 2.5mm centre-positive before use ✅

---

## Fusing

### 12 AWG Standard Inline Fuse Holder (£7.99, Amazon)
- **Role:** Compute domain inline fuse — 20 A blade fuse between 4S compute battery and BD-01/BD-02
- **Note:** Uses standard blade fuse (ATO/ATC). Fit a **20 A fuse** as per `fuses.md`. 12 AWG wire is correct for compute domain current levels.

### Inline Fuse Power Side (£12.99, Amazon)
- **Role:** 40 A ANL inline fuse holder — motive domain trunk fuse between 6S LiPo and WUPP fuse block
- **Note:** Fit the **40 A ANL fuse** supplied. Keep the spare ANL fuse accessible in the chassis. See `fuses.md` for full fuse rationale.

---

## Switches

### Switch for Main Battery — SCI A23-2 200A 12–50V DC (£14.95, Switch Electronics)
- **Role:** Main power cutoff switch for the **motive domain (6S LiPo, 22.2 V)**
- **Rated:** 200 A @ 12–50 V DC ✅ — well above the 40 A peak motive draw
- **Note:** Wire in series on the motive domain positive rail between XT90 and the 40 A ANL fuse holder

### Rocker Switch / Compute Domain Arming Switch (£8.99, Amazon)
- **Role:** Power cutoff switch for the **compute domain (4S battery, 14.8 V)**
- **Note:** Wire in series between the 4S battery positive and the 20 A inline fuse holder. Must be rated for ≥ 12 A at 14.8–16.8 V DC

---

## Safety

### E-Stop — Heavy Duty DC Emergency Stop (Amazon)
- **Rated:** 250 A @ 80 V DC
- **Action:** Push-to-break (pressing opens the circuit — cuts motive power), pull-to-make (pulling closes the circuit — restores power). Latches in open position until manually pulled to reset.
- **Terminals:** M8 copper primary connection posts
- **Protection:** IP50
- **Life:** 10,000 cycle mechanical design life
- **Mounting:** 2× M5 mounting holes
- **Role:** Cuts the motive domain (6S LiPo, 22.2 V) upstream of the 40 A ANL fuse and WUPP fuse block. Single-push disables all drivetrain and actuator power instantly.
- ⚠️ The compute domain (Jetson, LiDAR, router) remains **live through an E-stop** — compute battery is on a fully isolated rail. See `fuses.md`.

---

## Development & Diagnostic Tools

### USB Logic Analyzer (£9.98, Amazon)
- **Role:** Debug UART serial between Teensy and Sabertooth / BLD-510B; verify PWM signals
- **Note:** Most cheap USB logic analyzers use the Cypress FX2 chip and work with **PulseView / sigrok** on Linux/Windows. Connect GND first, then probe the signal line. Do not connect to voltages above 5 V without a voltage divider.

### Hall Ammeter Voltmeter (£10, Amazon)
- **Role:** In-line current and voltage monitoring — useful on the motive rail during testing to verify real current draw
- **Note:** Wire the shunt in series with the load being measured. Typically rated for a specific max current (e.g. 50 A or 100 A) — confirm rating covers your expected motive draw. Keep away from regolith dust.

---

## HATs & Breakout Boards

### Jetson Breakout Board (£11.80, Amazon)
- **Role:** Exposes the Jetson 40-pin GPIO header (J12) to breadboard/screw terminals
- **Note:** The 40-pin header on the Jetson is not currently allocated in INNEX-1 design. This board is useful for future expansion or debugging. Check it's compatible with the Orin Nano carrier board pinout (SP-11324-001 J12).

### Treedix Breakout Board Module (£18.20, Amazon)
- **Role:** Likely a Teensy 4.1 breakout / proto board for cleaner wiring
- **Note:** Likely a Teensy screw-terminal breakout board — makes motor controller wiring much cleaner than header pins alone.

### Teensy Stackable Header Kit (£1.80, PiHut)
- **Role:** Stackable pin headers for Teensy 4.1 — allows the Teensy to sit on a proto board or breakout while remaining removable
- **Note:** Solder headers to Teensy before installing in chassis. Ensures Teensy can be removed for firmware updates without desoldering.

---

## Spare / Contingency Items

### Logic Level Shifters ×2 (£10.78, Amazon) — **Spare**
- Not required in current design — Teensy 4.1 (3.3 V) is directly compatible with all motor controllers. Purchased as contingency. Store safely.

### PCA9685 Servo Driver Board (£7.32, Amazon) — **Removed from design**
- PCA9685 I2C expander was removed from the design — Teensy 4.1 has 31 native PWM pins. Keep as spare. Not connected in INNEX-1.

### Connector Kit (£7.99, Amazon)
- **Role:** Assorted connectors, Dupont crimp housings, jumper wires for signal-level connections
- **Note:** Useful for Teensy → motor controller signal wiring. When crimping Dupont connectors, use the correct crimper jaw for the terminal size — a bad Dupont crimp is one of the most common causes of intermittent faults.

---

## Flagged Items — Verify Before Assembly

| Item | Check |
|------|-------|
| Yellow Ring Terminal | Verify crimp barrel fits **10 AWG** wire — some "yellow" terminals are rated for 4–6 AWG only |
| XT90 Anti-Spark | Confirm male/female orientation — resistor pre-charge pin must be on battery side |
| DC Barrel Jack (5.5×2.5mm) | Confirm centre-positive, 12 V from BD-01 before plugging Jetson in |
| E-Stop | Push-break (250 A, 80 V DC) — wire in series on motive domain positive rail |
| Compute Switch | Confirm rating ≥ 12 A @ 16.8 V for compute domain arming switch |
| BD-01 Buck | Confirm module output current rating ≥ 10 A |

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-24 | eniomecaj | Initial file — sourced from INNEX-1 BOM (connectors/accessories section) |
| 2026-05-24 | eniomecaj | Updated BD-01/BD-02 names; WAGO 2-in-4-out marked active for BD-01 rail; barrel jack roles clarified; E-stop specs updated; switch domains corrected; KJ refs removed |
