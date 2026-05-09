# Foxglove Ground Control

Use `mission_control.layout.json` as the shared Mission Control layout. It is
deliberately lean: mission state, safety, diagnostics, drivetrain,
excavation, localisation, power telemetry, and one low-bandwidth camera view.

Import it in Foxglove from the layouts menu. Connect to the rover through
`foxglove_bridge`, usually:

```bash
ros2 run foxglove_bridge foxglove_bridge
```

Then open:

```text
ws://<jetson-ip>:8765
```

## Foxglove or RViz

Use Foxglove for operating the rover. It should answer the basic questions:

- What state is the mission in?
- Is motion inhibited or E-stop active?
- Are diagnostics OK, warning, stale, or error?
- Are drivetrain, excavation, localisation, and power telemetry sane?
- Is the front camera enough for situational awareness?

Use RViz for debugging navigation and perception. If you need TF, robot model,
costmap layers, point clouds, local planner behaviour, or camera geometry, use
RViz. Keep it closed during a scored run unless you need it; it is easy to turn
a quiet telemetry setup into a bandwidth problem.

The layout includes `/power/telemetry`. If the power telemetry PR has not
merged yet, that panel will simply be empty.
