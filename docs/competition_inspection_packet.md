# Competition Inspection Packet

The printable inspection and day-of compliance checklist travels with the
software. The wiki gives the broader reasoning.

The UK Lunabotics rulebook is the authority for the final numbers. Where older
NASA/UCF material differs, use the UK figure for this rover.

## Packet Contents

Bring these in printed or offline form:

- current UK Lunabotics rulebook;
- assigned SSID, router settings and radio settings;
- manufacturer datasheets for any Bluetooth or non-Wi-Fi radio part;
- power logger make, model, serial number, wiring position and readout steps;
- battery type, charger, storage and handling notes;
- robot mass, stowed dimensions, deployed dimensions and lifting point photos;
- autonomy sensor explanation and banned-sensor declaration;
- this repo's runtime profile output for `hardware_competition`;
- the Mission Control Foxglove layout and topic allowlist.

## E-stop And Motion Inhibit

The physical E-stop must latch. Releasing it must not resume rover motion.

Software mirrors that behaviour:

- `/safety/estop` reports the physical E-stop state.
- `/safety/motion_inhibit` latches the software inhibit.
- `/safety/reset_motion_inhibit` is the explicit reset path after E-stop is
  clear.

Inspection demo:

1. Command a low-speed movement.
2. Press E-stop and show motion stops.
3. Release E-stop and show motion does not resume.
4. Confirm `/safety/estop` is false.
5. Publish `/safety/reset_motion_inhibit`.
6. Confirm `/safety/motion_inhibit` is false before any further motion.

## Power Logger And Batteries

The COTS power logger is an inspection item. `/power/telemetry` is the ROS view
of the same power story, not a replacement for a physical logger.

The software low-voltage policy is deliberately conservative: warning voltage
appears as diagnostics WARN, critical voltage appears as diagnostics ERROR and
latched `/safety/motion_inhibit`. Re-enable motion only after the voltage has
recovered and `/safety/reset_motion_inhibit` has been published.

Document the logger wiring once electrical integration is final. The logger
needs to be visible or quickly readable by a judge, and its record needs to
survive E-stop.

Battery handling checklist:

- charge only while attended;
- unplug chargers overnight;
- store packs upright and separated;
- inspect dropped, hot, swollen or damaged packs;
- keep compute power and motive power assumptions written down;
- know where fire-safe storage is and who calls for help.

## Communications

The competition link is planned as 2.4 GHz Wi-Fi. Use 5 GHz only as an approved
fallback, not as the normal competition path.

Check before comm inspection:

- assigned SSID is visible;
- encryption is enabled;
- hidden network mode is off;
- 2.4 GHz channel width is `20 MHz`;
- the required comm-check channel is selected when instructed;
- average data use stays at or below `4,000 Kbps`;
- phone hotspots, tethering and spare backchannels are off;
- Bluetooth or non-Wi-Fi radios have printed manufacturer evidence.

Run the software profile check:

```bash
ros2 run lunabot_bringup runtime_profile check
ros2 run lunabot_bringup runtime_profile show --profile hardware_competition
```

## Autonomy Legality

Explain autonomy in one plain paragraph:

The rover uses onboard sensors and onboard software. It does not use arena
walls for mapping, localisation, autonomous navigation, collision avoidance or
autonomy sensing. It does not use GPS, compass heading, ultrasonic proximity
sensing, touch sensors for obstacle avoidance, or uploaded obstacle locations.

Allowed and intended autonomy inputs:

- wheel odometry;
- IMU data with no magnetic-heading dependency;
- AprilTag detection in the start zone;
- front and rear camera data for tag detection, operator view and close-range
  hazard evidence;
- OS1-128 LiDAR once mounted, after wall-region filtering;
- live costmaps generated from legal onboard sensor data.

Show these topics or views if asked:

- `/localisation/start_zone_status`;
- AprilTag detections or tag pose output;
- `/odometry/local`;
- filtered point-cloud or obstacle evidence;
- `/mission/autonomy_mode`;
- `/safety/motion_inhibit`;
- evidence pack manifest from `mission_evidence`.

## Mission Control Conduct

Mission Control is for mission-critical equipment only. Keep phones, tablets,
extra laptops, smart watches and personal radios out of the MCC.

Autonomy callouts:

1. Tell the Mission Control Judge which autonomy attempt is starting.
2. Start the attempt.
3. Keep hands off controls and laptops during the attempt.
4. Announce completion or failure before touching controls again.

At the end of the run, stop or inhibit further rover action first. Keep comms
and evidence intact until the judge releases the team.

## Related Docs

- `docs/hardware_week_runbook.md`
- `docs/jetson_runtime_profiles.md`
- `docs/mission_evidence_workflow.md`
- `docs/foxglove/README.md`
- `src/lunabot_safety/README.md`
- `src/lunabot_localisation/README.md`
- `src/lunabot_bringup/config/runtime_profiles.yaml`
