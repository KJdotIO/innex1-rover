# Jetson Runtime Profiles

The competition communications rule that matters most for software is simple:
average data utilisation must stay at or below **4,000 Kbps**. There is no peak
limit, but treating that as permission to stream raw sensors would be a small
masterclass in missing the point.

The default competition posture is:

- run autonomy and safety locally on the Jetson;
- expose only operator telemetry and health topics to Foxglove;
- keep raw images, point clouds, RViz, Gazebo and heavy rosbag captures off by
  default;
- record evidence locally with the minimal rosbag profile unless debugging
  requires more.

The machine-readable source of truth is:

```bash
ros2 run lunabot_bringup runtime_profile check
ros2 run lunabot_bringup runtime_profile show --profile hardware_competition
```

## Profiles

| Profile | Use it for | Default posture |
|---|---|---|
| `sim_debug` | Local simulation debugging | Visual tools and selected heavy topics allowed. |
| `sim_competition` | Dress rehearsal | Same telemetry rules as competition, with sim time. |
| `hardware_bringup` | Bench checks | Hardware bridges on, debug tools explicit, raw streams off. |
| `hardware_competition` | Competition run | Lean telemetry only, no raw sensor streams. |

## Foxglove Exposure

For `hardware_competition`, expose only:

- `/mission/state`
- `/mission/autonomy_mode`
- `/mission/time_remaining_s`
- `/mission/cycle_count`
- `/mission/last_failure_reason`
- `/safety/estop`
- `/safety/motion_inhibit`
- `/drivetrain/status`
- `/excavation/status`
- `/localisation/start_zone_status`
- `/diagnostics`

Do not expose raw camera images, point clouds, debug costmap panels or the heavy
rosbag profile during a scored run. If a raw stream is needed for diagnosis, use
`hardware_bringup` or `sim_debug`, say why, and turn it back off before the run.

## Measurement Commands

Run these on the Jetson during a representative dry mission:

```bash
ros2 run lunabot_bringup runtime_profile check
ros2 run lunabot_bringup runtime_profile show --profile hardware_competition --json
ros2 topic bw /mission/state
ros2 topic bw /drivetrain/status
ros2 topic bw /excavation/status
ros2 topic hz /diagnostics
top -b -n 1 | head -n 20
free -h
du -sh ~/innex1_mission_evidence 2>/dev/null || true
```

For Foxglove traffic, measure at the network edge as well as ROS:

```bash
ip -s link
sar -n DEV 1 10
```

If `sar` is not installed, use `ifstat`, `nload`, router counters, or the access
point dashboard. The number that matters is average utilisation on the robot
communications link, not only the estimated ROS payload.

## Launch Defaults

Competition launch arguments should match the checked profile:

```bash
ros2 launch lunabot_bringup navigation.launch.py \
  use_sim_time:=false \
  launch_rviz:=false \
  enable_teleop:=false \
  lidar_costmap_phase:=false \
  enable_apriltag_debug:=false
```

Evidence capture should stay lean:

```bash
ros2 run lunabot_bringup mission_evidence \
  --profile minimal \
  --label hardware-competition \
  -- ros2 launch lunabot_bringup mission_manager.launch.py
```

Use `debug` or `heavy` evidence profiles only for short investigations. They are
not the default scored-run posture.
