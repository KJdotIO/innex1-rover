# lunabot_control

Deposition hardware bridge and material-handling action servers for the
INNEX-1 rover.

## Hardware

- **Actuators**: 4x linear actuators — 1 door (hopper gate) + 2 bed
  (left/right tilt) + 1 spare.
- **Controllers**: 2x Cytron MDD10A in sign-magnitude mode (PWM + DIR),
  driven by Jetson GPIO.
- **Pin numbering**: BOARD convention. Defaults in `config/deposition.yaml`.

If Jetson GPIO is unavailable (e.g. running on a desktop) the bridge enters
**dry-run mode**: action goals are accepted and the phase state machine runs,
but no physical actuation occurs.

## Nodes

### `deposition_bridge`

ROS 2 action server for the hopper dump sequence. Runs on a
`MultiThreadedExecutor` to allow concurrent feedback publishing.

**Action**: `/mission/deposit` (`lunabot_interfaces/action/Deposit`)

Dump sequence phases:
1. **OPENING** — extend door actuator for `door_open_duration_s`.
2. **RAISING** — extend both bed actuators for `bed_raise_duration_s`.
3. **DUMPING** — hold raised for `dump_hold_duration_s`.
4. **CLOSING** — retract bed, then retract door.

Each phase is bounded to `_MAX_PHASE_DURATION_S` (30 s). Safety is checked
every feedback tick; if E-stop or motion inhibit activates mid-sequence, all
actuators stop and the goal aborts with `REASON_ESTOP`.

| Direction | Topic / Action | Type |
|-----------|----------------|------|
| Action | `/mission/deposit` | `lunabot_interfaces/action/Deposit` |
| Subscribe | `/safety/motion_inhibit` | `std_msgs/Bool` (transient-local) |
| Subscribe | `/safety/estop` | `std_msgs/Bool` |

### `material_action_server` / `material_action_client`

Stub nodes from the original material-handling scaffold. Not yet connected to
real hardware.

## Key files

- `config/deposition.yaml`: GPIO pin mapping, PWM frequency, duty cycle, and
  phase durations.
- `launch/material_actions.launch.py`: launch wrapper for the action nodes.

## Common failure modes

- GPIO permission errors on Jetson (`sudo groupadd -f -r gpio && sudo usermod -aG gpio $USER`).
- Phase durations too short — actuators don't fully extend/retract. Tune on
  real hardware.
- E-stop during dump leaves the bed partially raised; operator must manually
  lower or re-run the sequence after clearing E-stop.
