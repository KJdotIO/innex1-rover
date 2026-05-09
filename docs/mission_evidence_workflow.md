# Mission evidence workflow

This is the way we record runs that are worth keeping.

Most runs are not worth keeping. If the stack was half-launched, the rover was
already in a strange pose, or someone was still tuning parameters mid-run, do
not bless that bag as evidence. Keep it only if it explains a specific fault.

## Golden shuttle bag

The golden shuttle bag is a clean one-cycle sim run using the competition
checkpoint route:

1. start zone
2. mid-obstacle checkpoint at `(3.0, -1.1, 0.0)`
3. excavation target at `(6.0, -1.5, 0.0)`
4. mid-obstacle checkpoint again
5. deposition target at `(0.3, -2.8, 0.0)`

Those coordinates come from `src/lunabot_bringup/config/arena_waypoints.yaml`.
The midpoint is deliberate. It keeps the mission route honest about the
obstacle zone instead of asking Nav2 to solve one big vague goal.

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

The pack lands under `~/innex1_mission_evidence/`. It should contain:

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
scan, map, costmaps, and action status.

Use `heavy` only for short perception investigations. It records raw image and
point-cloud topics and can burn disk quickly. That is fine when you mean it.
It is annoying when you do it by accident.
