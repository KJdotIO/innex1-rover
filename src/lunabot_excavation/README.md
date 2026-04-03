# Excavation Bench Tooling

This package includes a small bench-facing path for the excavation subsystem.

The important bit is that the bench path still goes through the excavation controller. It does
not bypass the controller and poke raw hardware commands directly.

## Launch

Start the controller for bench work:

```bash
ros2 launch lunabot_excavation excavation_bench.launch.py
```

Start the excavation sim proxy, controller, and action adapter together:

```bash
ros2 launch lunabot_excavation excavation_sim.launch.py
```

`excavation_sim.launch.py` already includes the excavation action adapter. The broader
`material_actions.launch.py` path does not start `excavation_sim_proxy`, so it is not a
drop-in replacement for excavation simulation by itself. Use it only when you want the broader
action path and are providing excavation telemetry separately:

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

## Sim Proxy Fault Injection

The excavation sim launch exposes three simple fault knobs:

```bash
ros2 launch lunabot_excavation excavation_sim.launch.py force_overcurrent:=true
ros2 launch lunabot_excavation excavation_sim.launch.py force_driver_fault:=true
ros2 launch lunabot_excavation excavation_sim.launch.py hold_home_switch_false:=true
```

- `force_overcurrent` faults once the fake motor starts, which lets the controller/action path
  exercise the jam or overcurrent branch.
- `force_driver_fault` publishes a persistent driver fault from the fake hardware.
- `hold_home_switch_false` prevents homing completion, which gives a deterministic timeout path.

For a simple fault-path check, stop the controller or block the service path and confirm the
bench CLI exits cleanly with an `error:` message rather than a traceback.

## Notes

- Jog is forward-only in this first pass.
- Jog is bounded by duration and only accepted when the controller is `READY`.
- Stop still wins.
- Faults still latch in the controller and must be cleared there.
