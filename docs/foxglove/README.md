# Foxglove Ground Control

Use `mission_control.layout.json` as the shared Mission Control layout. It shows
mission state, safety, diagnostics, drivetrain, excavation, localisation, power
telemetry, and one compressed front camera view.

Import it in Foxglove from the layouts menu. Connect to the rover through
`foxglove_bridge` through the ground-control launch file:

```bash
ros2 launch lunabot_bringup foxglove_ground_control.launch.py \
  profile:=sim_competition \
  use_sim_time:=true
```

Then open:

```text
ws://<jetson-ip>:8765
```

For hardware competition mode, use:

```bash
ros2 launch lunabot_bringup foxglove_ground_control.launch.py \
  profile:=hardware_competition \
  use_sim_time:=false
```

The launch file republishes `/camera_front/image` to
`/camera_front/image/compressed` using JPEG compression. Foxglove must subscribe
to the compressed topic. Do not point the operator layout at raw
`/camera_front/image`.

On the Jetson bench test, the usable live view was the lean path: throttle the
front RGB image to about 8 Hz, compress that stream, and keep Foxglove to the
camera plus small telemetry. Full-rate RGB, compressed depth and point clouds
made Foxglove fall several seconds behind.

For a camera-only operator check, use `oak_camera_lean.layout.json`. It shows
the front compressed image, camera info and diagnostics. Keep only one Foxglove
tab connected while testing, otherwise the bridge sends the same video stream to
each client.

Rear camera streaming is available when needed, but is off by default:

```bash
ros2 launch lunabot_bringup foxglove_ground_control.launch.py \
  profile:=hardware_bringup \
  use_sim_time:=false \
  enable_rear_camera:=true
```

## Foxglove or RViz

Use Foxglove for operating the rover. The operator view answers the basic questions:

- What state is the mission in?
- Is motion inhibited or E-stop active?
- Are diagnostics OK, warning, stale, or error?
- Are drivetrain, excavation, localisation, and power telemetry sane?
- Is the front camera enough for situational awareness?

Use RViz, or a separate Foxglove debug profile, for navigation and perception
debugging. If you need TF, robot model, costmap layers, point clouds, local
planner behaviour, or camera geometry, keep that out of the scored-run operator
layout.

`oak_camera_debug.layout.json` is intentionally heavy. It exists for short bench
checks only. Do not use it as the normal driving view.

## Ouster LiDAR Debug

Use `ouster_lidar_debug.layout.json` for a live Ouster-only debug view. This is
for bring-up and mapping checks, not for the scored-run operator layout.

Connect the Ouster to the Jetson Ethernet port, then check the Jetson Ethernet
address:

```bash
ip -br addr show
```

If the Ethernet address is not `169.254.86.134`, edit `udp_dest` in
`src/lunabot_bringup/config/ouster_lidar_debug.yaml` or pass a copied parameter
file with the correct address. Then start the debug stack:

```bash
ros2 launch lunabot_bringup ouster_lidar_foxglove_debug.launch.py
```

Open Foxglove on the ground-control laptop and connect to:

```text
ws://<jetson-tailscale-or-router-ip>:8765
```

The debug launch exposes `/ouster/points`, `/ouster/imu`, `/ouster/scan`,
`/ouster/metadata`, `/ouster/telemetry`, and TF. Keep this separate from the
competition layout because point clouds are too heavy for the 4,000 Kbps
telemetry budget.

The layout includes `/power/telemetry`. If the power telemetry PR has not
merged yet, that panel will simply be empty.

## Bandwidth

The rulebook limit is **4,000 Kbps average**. Raw image and point cloud topics
must not be streamed to Mission Control. A single `640x480` raw RGB image stream
at `15 Hz` can exceed `100,000 Kbps`, before WebSocket overhead.

The competition profile budgets for one compressed camera stream at roughly
`1,500 Kbps`, plus low-rate operator telemetry. That leaves more than `1,000
Kbps` of margin under the rulebook limit.
