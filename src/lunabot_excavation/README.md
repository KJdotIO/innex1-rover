# Excavation Bench Tooling

This package includes a small bench-facing path for the excavation subsystem.

The important bit is that the bench path still goes through the excavation controller. It does
not bypass the controller and poke raw hardware commands directly.

## Launch

Start the controller for bench work:

```bash
ros2 launch lunabot_excavation excavation_bench.launch.py
```

If you want the mission-facing action adapter as well, use:

```bash
ros2 launch lunabot_control material_actions.launch.py
```

## Bench Commands

Print one excavation status snapshot:

```bash
ros2 run lunabot_excavation excavation_bench status
```

Request homing:

```bash
ros2 run lunabot_excavation excavation_bench home
```

Request a normal start:

```bash
ros2 run lunabot_excavation excavation_bench start
```

Request a bounded forward jog:

```bash
ros2 run lunabot_excavation excavation_bench jog-forward --duration 0.5
```

Stop the subsystem:

```bash
ros2 run lunabot_excavation excavation_bench stop
```

Clear a latched fault:

```bash
ros2 run lunabot_excavation excavation_bench clear-fault
```

## Notes

- Jog is forward-only in this first pass.
- Jog is bounded by duration and only accepted when the controller is `READY`.
- Stop still wins.
- Faults still latch in the controller and must be cleared there.
