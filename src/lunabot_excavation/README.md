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

That launch now includes an explicit `excavation_telemetry_mock` node by default. It is a bench
mock, not real hardware IO, and it is there so the controller and action adapter can complete a
clean cycle on their own. If you already have a real telemetry source, pass
`use_mock_telemetry:=false`.

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

This is the full excavation start path, not a bounded jog. It will keep running until you
explicitly stop it or the controller faults.

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

## Suggested Bench Sequence

Bring the subsystem up in this order:

1. Start the bench launch.
2. Run `status` and confirm the controller is alive.
3. Run `home`.
4. Run `status` again and confirm `state: ready`.
5. Run one short `jog-forward --duration 0.5`.
6. Run `status` and confirm the controller returns to `ready` or `idle`.
7. If you use `start`, follow it with an explicit `stop`.

Expected state flow for a healthy bounded jog is:

- `ready`
- `starting`
- `excavating`
- `stopping`
- `ready` or `idle`

For a simple fault-path check, stop the controller or block the service path and confirm the
bench CLI exits cleanly with an `error:` message rather than a traceback.

## Material Bench Path

`material_actions.launch.py` is the standalone bench path for excavation plus the deposition
stub. It launches:

- the excavation telemetry mock
- the excavation controller
- the excavation action server
- the material action server

That means you can run a full material smoke check without embedded hardware:

```bash
ros2 launch lunabot_control material_actions.launch.py
ros2 run lunabot_control material_action_client
```

If you want to force a stop-side fault for readability checks, override the mock parameter:

```bash
ros2 launch lunabot_control material_actions.launch.py \
  fault_on_stop_code:=2
```

If you are wiring in a real `/excavation/telemetry` source, disable the mock explicitly:

```bash
ros2 launch lunabot_control material_actions.launch.py \
  use_mock_telemetry:=false
```

## Notes

- Jog is forward-only in this first pass.
- Jog is bounded by duration and only accepted when the controller is `READY`.
- Stop still wins.
- Faults still latch in the controller and must be cleared there.
