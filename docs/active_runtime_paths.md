# Active Runtime Paths

The launch paths below are current enough for hardware-week work. If a launch
file is not listed here, treat it as package-local debugging or implementation
detail until it is added deliberately.

The map follows the current repo shape rather than an ideal ROS 2 layout. The
aim is to make the active paths obvious before adding more hardware-facing code.

## Sim Navigation Check

Use this when checking Nav2, localisation, crater detection, and RViz against
the Gazebo yard.

```bash
ros2 launch lunabot_simulation moon_yard.launch.py
ros2 launch lunabot_bringup navigation.launch.py
```

`navigation.launch.py` includes:

- `lunabot_bringup/localisation.launch.py`
- `lunabot_bringup/nav2_navigation.launch.py`
- `lunabot_perception/crater_detection`
- `lunabot_bringup/navigate_to_pose_gate`
- optional joystick teleop and `twist_mux`
- optional RViz using `src/lunabot_bringup/rviz/navigation.rviz`

Do not launch `localisation.launch.py` separately for normal navigation checks.
Use it only when isolating localisation.

## Sim Mission Evidence

Use this when recording a known mission route for later review:

```bash
ros2 run lunabot_bringup mission_evidence \
  --profile minimal \
  --label golden-shuttle \
  --use-sim-time \
  -- ros2 launch lunabot_bringup mission_shuttle_evidence.launch.py \
    launch_rviz:=false \
    max_shuttle_cycles:=1
```

`mission_shuttle_evidence.launch.py` is simulation-only. It starts Gazebo,
navigation, simulated excavation, the deposition action server, E-stop bridge,
diagnostics, and one mission-manager cycle. It also disables the AprilTag
readiness gate because the sim does not always provide a stable start-zone tag
lock.

## Mission Manager Only

Use this only when the rest of the stack is already running:

```bash
ros2 launch lunabot_bringup mission_manager.launch.py
```

This launches the mission manager with the arena waypoint config. It does not
start Nav2, mechanisms, diagnostics, or safety nodes.

## Hardware Drivetrain Bring-Up

Use this for first-motion and bench drivetrain checks:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py max_throttle:=0.2
```

`drivetrain_bench.launch.py` starts:

- `lunabot_drivetrain/drivetrain_bridge`
- `lunabot_drivetrain/velocity_gate`
- optional joystick teleop through `twist_mux`

The command path is:

```text
/cmd_vel_teleop or /cmd_vel_nav
  -> twist_mux
  -> /cmd_vel_safe
  -> velocity_gate
  -> /cmd_vel_gated
  -> drivetrain_bridge
```

Keep the velocity gate in this path for hardware work. Do not command the
drivetrain directly unless you are running a controlled bench test and have a
stop plan.

## Excavation And Deposition Bring-Up

Use the package launch files for subsystem work:

```bash
ros2 launch lunabot_excavation excavation_bench.launch.py
ros2 launch lunabot_excavation excavation_sim.launch.py
ros2 launch lunabot_control material_actions.launch.py
```

For full mission simulation, use `mission_shuttle_evidence.launch.py` instead
of manually composing these pieces.

## Operator Telemetry

Use Foxglove for ground-control state and RViz for navigation debugging.

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
ros2 run rviz2 rviz2
```

Foxglove should show the low-bandwidth operator topics from
`runtime_profiles.yaml`. RViz should be opened when checking TF, robot model,
costmaps, point clouds, and Nav2 behaviour.

The Foxglove Bridge is the expected live ROS 2 bridge because it supports ROS 2
message definitions, graph introspection, and low-overhead WebSocket transport.
Diagnostics should continue to use `/diagnostics` with
`diagnostic_msgs/DiagnosticArray`, which is the standard ROS diagnostics topic
shape.

## Package-Local Debug Paths

These are still useful, but they are not the default hardware-week path:

| Path | Use |
|---|---|
| `lunabot_bringup/mission_dry_run.launch.py` | CI-style sim harness for bounded travel/action checks. |
| `lunabot_bringup/rover_diagnostics.launch.py` | Diagnostics-only run when the rest of the stack is already up. |
| `lunabot_excavation/excavation_controller.launch.py` | Controller-only excavation debugging. |
| `lunabot_excavation/excavation_action_server.launch.py` | Action-server-only excavation debugging. |
| `lunabot_teleop/joystick_teleop.launch.py` | Controller input debugging without drivetrain bridge. |

## Current Simplification Notes

- `lunabot_perception` is active because `navigation.launch.py` starts
  `crater_detection`.
- `lunabot_safety` is active because mission evidence and hardware motion use
  the E-stop to motion-inhibit bridge.
- `lunabot_navigation` owns Nav2 config and behaviour tree assets, not custom
  navigation nodes.
- The old `src/lunabot_bringup/config/crater_detection.rviz` file was removed
  because it was not referenced by docs or launch files and was not installed by
  the package. The active RViz config is
  `src/lunabot_bringup/rviz/navigation.rviz`.
