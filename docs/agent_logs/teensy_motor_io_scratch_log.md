# Teensy Motor IO Scratch Log

This log records decisions, tradeoffs, checks, and follow-up questions from the
agent-led Teensy motor IO implementation work. It is intentionally plain and
chronological so future reviewers can see what was assumed rather than having to
dig through chat.

## 2026-05-24 — Kickoff

- Branch created: `feature/teensy-motor-io`.
- Goal: build the Teensy-based low-level motor IO path as far as possible before
  the Teensy hardware is available, with desktop tests, simulation/fake serial,
  Jetson validation, docs, and Linear tracking.
- Local repo was clean and up to date with `origin/main` before branching.
- SSH to the Jetson via Tailscale works non-interactively:
  `ssh innex1-desktop ...` returned hostname `innex1-desktop`.
- Local tools present: `gh`, `tailscale`, `ssh`, `arduino-cli`, `cmake`, `g++`,
  Python 3.14.
- Jetson tools observed: `git`, `colcon`; `ros2` was not printed by the quick
  probe, so the Jetson ROS environment may require sourcing before use.
- Linear project found: `INNEX1 Rover` (`INX`).
- Existing related Linear issues found:
  - `INX-5` — Teensy encoder acquisition and wheel odometry path.
  - `INX-65` — next hardware integration plan.
  - `INX-69` — Jetson OS1/localisation timing handoff.

### Initial Direction

- Use a small custom binary protocol over USB CDC serial between Jetson and
  Teensy, with explicit framing and checksums.
- Keep firmware logic hardware-independent where possible, with a thin Arduino
  adapter later. This lets us test safety, packet parsing, command expiry,
  drive/excavation mutual exclusion, and telemetry without the Teensy.
- Treat the Teensy as the final authority before any hardware moves. ROS can
  request motion; the Teensy must still deny motion on stale heartbeat, E-stop,
  inhibit, ALM, invalid packet, unsafe mode, or missing startup configuration.

### Open Risks

- Need to confirm exact Sabertooth 2x32 packet/plain-text commands for current
  limit and ramping before implementing hardware-write behaviour. If the current
  limit is configured via DEScribe only, firmware must at least verify and expose
  a clear preflight state.
- Need to settle E-stop interpretation against UK Lunabotics RuleBook 2026 v1.0:
  current docs say compute stays live through E-stop, but rulebook wording says
  the E-stop disconnects batteries from all controllers and isolates active
  subsystems. This is a design/inspection question, not just software.
- Current repo drivetrain code still assumes Jetson-to-Sabertooth UART. The new
  Teensy architecture should avoid breaking existing bench workflows until the
  Teensy path is ready.

## 2026-05-24 — First Implementation Slice

- Created Linear subtickets `INX-71` through `INX-76` under `INX-70`.
- Added a host-testable firmware core under `firmware/teensy_motor_io`:
  COBS framing, CRC-16/CCITT-FALSE, protocol decode/encode tests, and a
  fail-closed safety state machine.
- Added Jetson-side Teensy protocol and serial client modules inside
  `lunabot_drivetrain`, plus a fake Teensy pseudo-terminal endpoint for tests.
- Added `teensy_drivetrain_bridge` while keeping the old `drivetrain_bridge`
  as the default direct Sabertooth bench path. The launch file now uses
  `bridge_backend:=sabertooth|teensy`.
- Trade-off: kept the ROS-side bridge in `lunabot_drivetrain` rather than
  creating a new ROS package. This preserves the existing `/cmd_vel_gated`,
  `/drivetrain/status`, `/drivetrain/telemetry`, `/odom_wheels`, and
  `/joint_states_wheels` contracts.
- Tightened safety merge logic so Teensy telemetry can assert E-stop/inhibit
  but cannot clear the ROS E-stop or latched motion-inhibit state.
- Sabertooth research update: field firmware should use modern Packet Serial
  with CRC. Plain Text Serial is bench/debug only. Current limiting and ramping
  should be configured in DEScribe first; runtime current-target commands are a
  later improvement, not the first safety line.
- Local validation run:
  - `cmake --build build/teensy_motor_io && ctest --test-dir build/teensy_motor_io --output-on-failure` passed.
  - `PYTHONPATH=src/lunabot_drivetrain uv run --with pytest python -m pytest src/lunabot_drivetrain/test/test_teensy_protocol.py src/lunabot_drivetrain/test/test_teensy_serial.py -q` passed.
  - Full local `test_sabertooth_serial.py` still needs a ROS environment because
    this macOS Python does not provide `rclpy`; run that on the Jetson.

## 2026-05-24 — Jetson Validation

- Jetson repo had unrelated OAK-D bringup changes in the main worktree. Stashed
  them before branch work with a `codex-pre-teensy-motor-io-*` stash entry.
- Fast-forwarded Jetson `main`, created `feature/teensy-motor-io`, and copied
  only this branch's changed files into that worktree.
- Jetson validation passed:
  - `cmake -S firmware/teensy_motor_io -B build/teensy_motor_io && cmake --build build/teensy_motor_io && ctest --test-dir build/teensy_motor_io --output-on-failure`
  - `PYTHONPATH=src/lunabot_drivetrain python3 -m pytest src/lunabot_drivetrain/test/test_teensy_protocol.py src/lunabot_drivetrain/test/test_teensy_serial.py -q`
  - `colcon build --symlink-install --packages-select lunabot_interfaces lunabot_drivetrain`
  - `colcon test --packages-select lunabot_drivetrain --event-handlers console_direct+`
- `colcon test` result on Jetson: `46 passed, 2 warnings`. The warnings are
  existing pytest/flake8 selectable-groups deprecation warnings, not failures.
- Fake Teensy ROS smoke test passed after fixing the PTY fake to use raw mode
  and adding a Teensy startup grace period:
  - one `teensy_drivetrain_bridge` node running against the fake PTY;
  - `/drivetrain/status.state` reported `1` (`READY`);
  - `/drivetrain/telemetry.controller_online` reported `[True, True]`.
- Debug note: earlier smoke attempts left stale bridge child processes because
  `ros2 run` spawns the real executable underneath the wrapper. Killed those and
  reran the smoke test using the installed executable directly so cleanup owned
  the actual bridge process.
