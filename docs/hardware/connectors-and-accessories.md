# INNEX-1 Connectors & Accessories Reference

Single reference for all connector, passive, and accessory items purchased. Not individual
component datasheets — just enough to know what each item is for and any gotchas.

---

## Power Distribution & Regulation

### BD-01 — 12 V Buck Converter (Voltage Regulator 2, £7.77, Amazon)
- **Role:** Steps 14.8 V compute battery down to 12 V for Jetson Orin Nano + OS1 LiDAR
- **Rated:** ≥ 10 A output required (Jetson ~3–4 A + LiDAR ~2–3 A = ~6 A combined, 10 A headroom)
- **Note:** Pending CDR update — confirm module is rated ≥ 10 A and update power budget table

### BD-02 — 5 V Buck Converter (Voltage Regulator 1, £3.22, Switch Electronics)
- **Role:** Steps 14.8 V compute battery down to 5 V for OAK-D Pro cameras (×2) + GL-A1300 router
- **Rated:** Must supply ~3–4 A total (2 × camera ~0.5–1 A + router ~1.3 A)
- **Note:** Output goes to WAGO 2-in-6-out block; no USB hub needed

---

## WAGO Connectors

### WAGO 2-in-6-out (£5.53, Amazon)
- **Role:** BD-02 output distribution — 5 V to cameras ×2 and GL-A1300 router (3 active outputs, 3 spare)
- **Status:** ✅ This is the current design — replaces the 2-in-4-out block
- ⚠️ Ensure wire gauge matches WAGO clamp rating (check product page — typically rated for 0.5–6 mm² / 20–10 AWG)

### WAGO 2-in-4-out (£9.89, Amazon)
- **Role:** Original BD-02 distribution block — superseded by 2-in-6-out
- **Status:** Spare / not used in final design. Keep as backup.

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
- ⚠️ Use the correct terminal size for 10 AWG wire — yellow ring terminals are typically rated for 4–6 AWG in some kits. **Verify the crimp barrel fits 10 AWG before use.**

### USB-C to 2-Pin Bare Wire (×2, £8.99, Amazon)
- **Role:** 5 V power feed cables from BD-02 WAGO → GL-A1300 router (USB-C) and any USB-C powered device
- **Note:** Verify polarity of bare wires before connecting to WAGO — red = +5 V, black = GND. USB-C can be damaged by reverse polarity.

### 6× 5.5mm × 2.5mm 90° DC Power Male Plug (£6.99, Amazon)
- **Role:** DC barrel jack connectors for Jetson Orin Nano power input (J16 on carrier board)
- **Note:** Jetson DC jack is **5.5mm OD × 2.5mm ID center-positive** — this matches the item spec ✅. The 90° angle helps with cable routing in the chassis. Solder the wire to the plug before heatshrinking — centre pin = positive (12 V), outer shell = GND.

### Pigtail to DC Barrel Jack (£7.19, Amazon)
- **Role:** Pre-made pigtail cable for Jetson power — alternative/backup to the bare plugs above
- **Note:** Check that it's also 5.5mm × 2.5mm centre-positive before use.

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

### Switch for Main Battery (£14.95, Switch Electronics)
- **Role:** Main power cutoff switch for the compute domain (4S battery)
- **Note:** Rated current must cover BD-01 + BD-02 combined draw (~8 A max). Verify rating on the purchased switch. Wire in series between the 4S battery positive and the 20 A inline fuse holder.

### Rocker Switch (£8.99, Amazon)
- **Role:** Secondary power switch — likely for motive domain or as a panel power indicator switch
- **Note:** Clarify with KJ which domain this serves. If on the motive domain, verify it's rated for 22.2 V DC at the expected current.

---

## Safety

### E-Stop (£32.95, Amazon)
- **Role:** Physical emergency stop — cuts motive power during a fault or unsafe condition
- **Wiring options:**
  1. **Hardware E-stop via Sabertooth:** Wire E-stop contacts to Sabertooth A1 (M1 enable) and A2 (M2 enable). Set Sabertooth DIP switch 6 to OFF (emergency stops enabled). When E-stop is pressed, A1/A2 go low → both motor outputs cut immediately.
  2. **Main contactor / relay:** E-stop cuts the motive rail entirely upstream of the WUPP fuse block.
- ⚠️ The compute domain (Jetson, LiDAR, router) should **remain live through an E-stop** per `fuses.md` — only motive power should be interrupted.
- Confirm wiring approach with KJ before competition — option 1 (Sabertooth A1/A2) is already supported in hardware with no additional relay needed.

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
- **Note:** Clarify with KJ what this is specifically for. If it's a Teensy screw-terminal breakout, it makes the motor controller wiring much cleaner than header pins alone.

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
| E-Stop | Agree wiring method with KJ — Sabertooth A1/A2 hardware path recommended |
| Rocker Switch | Confirm voltage/current rating matches its domain |
| BD-01 Buck | Confirm module output current rating ≥ 10 A |

---

## Revision History

| Date | Author | Change |
|------|--------|--------|
| 2026-05-24 | eniomecaj | Initial file — sourced from INNEX-1 BOM (connectors/accessories section) |
