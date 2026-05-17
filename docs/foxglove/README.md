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

The layout includes `/power/telemetry`. If the power telemetry PR has not
merged yet, that panel will simply be empty.

## Bandwidth

The rulebook limit is **4,000 Kbps average**. Raw image and point cloud topics
must not be streamed to Mission Control. A single `640x480` raw RGB image stream
at `15 Hz` can exceed `100,000 Kbps`, before WebSocket overhead.

The competition profile budgets for one compressed camera stream at roughly
`1,500 Kbps`, plus low-rate operator telemetry. That leaves more than `1,000
Kbps` of margin under the rulebook limit.
