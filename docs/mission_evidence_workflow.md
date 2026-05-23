# Mission evidence workflow

Mission evidence records runs that need later review.

Do not keep every run as evidence. If the stack was half-launched, the rover was
already in an unusual pose, or parameters were changing mid-run, keep the bag
only when it explains a specific fault.

Evidence bags are not autonomy inputs. Do not use a bag, replay, screenshot or
operator observation to upload judged obstacle locations into a later autonomy
attempt. A bag can prove what the rover saw and did; it must not become a route
planning shortcut after the field has been seen.

## Golden shuttle bag

The golden shuttle bag is a clean one-cycle sim run using the competition
checkpoint route:

1. start zone
2. mid-obstacle checkpoint at `(3.0, -1.1, 0.0)`
3. excavation target at `(6.0, -1.5, 0.0)`
4. mid-obstacle checkpoint again
5. deposition target at `(0.3, -2.8, 0.0)`

Those coordinates come from `src/lunabot_bringup/config/arena_waypoints.yaml`.
The midpoint is deliberate. It makes the route pass through the obstacle-zone
checkpoint instead of sending Nav2 one long start-to-zone goal.

The launch is simulation-only. It starts Nav2 directly and bypasses the
AprilTag readiness gate because the moon yard sim does not always provide a
stable start-zone tag lock. It also waits before starting the mission so Nav2
has time to finish its lifecycle bringup on the Jetson. Do not copy those
assumptions into hardware bringup.

Record it on the Jetson:

```bash
ros2 run lunabot_bringup mission_evidence \
  --profile minimal \
  --label golden-shuttle \
  --use-sim-time \
  -- ros2 launch lunabot_bringup mission_shuttle_evidence.launch.py \
    launch_rviz:=false \
    max_shuttle_cycles:=1
```

For the legal LiDAR-odometry dress rehearsal, use the same launcher with two
or three cycles and the KISS-ICP baseline enabled:

```bash
ros2 run lunabot_bringup mission_evidence \
  --profile debug \
  --label lio-shuttle-3-cycle \
  --use-sim-time \
  -- ros2 launch lunabot_bringup mission_shuttle_evidence.launch.py \
    launch_rviz:=false \
    ouster_vertical_samples:=128 \
    lidar_odometry_backend:=kiss_icp \
    max_shuttle_cycles:=3
```

That run is only credible if `/localisation/lidar/points_legal` is present,
`/localisation/lidar/odometry` is active, and the legal LiDAR filter diagnostics
show wall/out-of-field rejection before the odometry backend consumes the cloud.

A complete pack lands under `~/innex1_mission_evidence/` and contains:

- `bag/`
- `logs/mission_command.log`
- `logs/rosbag_record.log`
- `config/arena_waypoints.yaml`
- `launch/mission_shuttle_evidence.launch.py`
- `manifest.json`

Check the bag before calling it golden:

```bash
ros2 bag info ~/innex1_mission_evidence/<pack>/bag
```

Then open `manifest.json`. The useful bit should look like this:

```json
{
  "results": {
    "mission_summary": {
      "completed_cycles": 1,
      "failure_reason": "",
      "overall": "pass"
    }
  }
}
```

If `overall` is not `pass`, the bag is not a golden bag. It may still be useful
as a fault bag, but name it that way.

## Replay

Replay the bag with:

```bash
ros2 bag play ~/innex1_mission_evidence/<pack>/bag
```

Use replay for inspection first. Do not assume replay is a drop-in replacement
for every sim run. It is best for checking topics, diagnostics, mission state,
TF, command output, and timing around a known run.

## Profiles

Use `minimal` for normal mission evidence. It records operator state, safety
state, diagnostics, status topics, TF, and command topics.

Use `debug` when a run failed and you need navigation state. It adds odometry,
scan, map, costmaps, action status, LiDAR-odometry output, and wall-excluded
point-cloud topics.

Use `heavy` only for short perception investigations. It records raw image and
point-cloud topics alongside the filtered autonomy inputs and can use disk
space quickly.

## Sensor Claims

Each competition evidence pack should be understandable without a long verbal
explanation. The manifest should make these claims clear, either directly or in
the run note:

- autonomy used onboard sensor data and onboard software;
- arena walls were not used for mapping, localisation, autonomous navigation or
  collision avoidance; wall-capable point clouds were filtered before Nav2,
  collision monitor, crater detection, and any LiDAR odometry backend consumed
  them;
- no GPS, compass heading, ultrasonic proximity sensing or touch sensing for
  obstacle avoidance was used;
- no obstacle-location upload was made after seeing the arena;
- Mission Control telemetry was passive and did not create an extra command
  path during hands-free autonomy.
