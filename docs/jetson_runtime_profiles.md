# Jetson Runtime Profiles

The competition communications rule that matters most for software is simple:
average data utilisation must stay at or below **4,000 Kbps**. There is no peak
limit. Older NASA/UCF public material has used **5,000 Kbps**, but the UK
rulebook is stricter, so this repo uses **4,000 Kbps**. We still keep the
average low because the operator link needs to stay stable.

The default competition posture is:

- run autonomy and safety locally on the Jetson;
- expose only operator telemetry and health topics to Foxglove;
- stream camera video only through compressed image topics;
- keep raw images, point clouds, RViz, Gazebo and heavy rosbag captures off by
  default;
- record evidence locally with the minimal rosbag profile unless debugging
  requires more.

This profile is also part of the comm-check story: Mission Control should show
only robot-originating telemetry/video and the official competition monitors.
Raw debug streams are useful in the lab, but they are not the scored-run
operator view.

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
- `/power/telemetry`
- `/camera_front/image/compressed`
- `/camera_front/camera_info`

Do not expose raw camera images, depth images, point clouds, debug costmap
panels or the heavy rosbag profile during a scored run. If a raw stream is
needed for diagnosis, use `hardware_bringup` or `sim_debug`, say why, and turn
it back off before the run.

Do not add phone hotspots, tethering, spare laptops or browser-based
backchannels to work around this allowlist. If the operator cannot see enough
through the competition profile, fix the profile deliberately before the run.

Start Foxglove through the project launch file so the allowlist and camera
compression stay consistent:

```bash
ros2 launch lunabot_bringup foxglove_ground_control.launch.py \
  profile:=hardware_competition \
  use_sim_time:=false
```

For simulation:

```bash
ros2 launch lunabot_bringup foxglove_ground_control.launch.py \
  profile:=sim_competition \
  use_sim_time:=true
```

The front camera is republished as `/camera_front/image/compressed` with JPEG
quality `50` by default. The rear camera republisher is available with
`enable_rear_camera:=true`, but keep it off unless you need it.

## Measurement Commands

Run these on the Jetson during a representative dry mission:

```bash
ros2 run lunabot_bringup runtime_profile check
ros2 run lunabot_bringup runtime_profile show --profile hardware_competition --json
ros2 topic bw /mission/state
ros2 topic bw /drivetrain/status
ros2 topic bw /excavation/status
ros2 topic bw /camera_front/image/compressed
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

Keep evidence capture lean:

```bash
ros2 run lunabot_bringup mission_evidence \
  --profile minimal \
  --label hardware-competition \
  -- ros2 launch lunabot_bringup mission_manager.launch.py
```

Use `debug` or `heavy` evidence profiles only for short investigations. They are
not the default scored-run posture.

## Navigation Debug

When the rover gets close to craters or boulders, use a debug profile instead of
adding costmaps and point clouds to the operator layout. The useful debug topics
are `/global_costmap/costmap`, `/local_costmap/costmap`, `/map`, paths, TF and
collision monitor polygons. Do not stream those at the same time as camera video
unless you are doing a short, supervised investigation.
