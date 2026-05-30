# Remote Support Workflow

Use this before remote repo work, review work, or competition-readiness changes.

The short version: start from the rover context, then make the change. Robots
punish guesswork.

## Sources Of Truth

Start every serious remote-support task with these files:

- `AGENTS.md`
- `README.md`
- `ROVER_CODING_STANDARD.md`
- `.github/contracts/interface_contracts.json`
- `docs/active_runtime_paths.md`
- `docs/hardware_week_runbook.md`
- `docs/competition_remote_support.md`
- package READMEs under `src/*/README.md`

If a local `wiki/` checkout exists, include the relevant wiki page too. If the
wiki is not checked out, say so in the task prompt rather than pretending the
context is available.

## Installed Robotics Skills

If local robotics skills are available, use the matching skill before editing or
reviewing. This repo does not include the skill files.

- `ros2-development` for ROS 2 Humble, `colcon`, launch files, parameters,
  QoS, actions, Nav2, DDS, package metadata and build issues.
- `robotics-design-patterns` for fail-closed behaviour, watchdogs, safety
  architecture, hardware abstraction, state machines and sim-to-real choices.
- `robotics-software-principles` for robotics code reviews, module boundaries,
  hardware interfaces, configuration and risky refactors.
- `robotics-testing` for ROS tests, launch tests, mock hardware, deterministic
  replay, CI and hardware-in-the-loop planning.
- `robot-perception` for OAK-D, depth, AprilTags, Ouster/LiDAR, point clouds,
  TF/extrinsics, calibration and sensor timing.
- `robotics-security` for SSH, router/network exposure, secrets, certificates,
  Docker/devcontainer hardening, DDS security and E-stop security.

If a skill is missing, say so and continue from the repo docs.

## Good Task Prompts

Good tasks are bounded and testable:

```text
Inspect the drivetrain bridge and velocity gate for fail-closed behaviour.
Only report issues that could move real hardware unsafely.
Use AGENTS.md, docs/teensy_drivetrain_bringup.md, and the interface contracts.
Return file references and a smallest safe fix.
```

```text
Update the Ouster TF docs so sim and hardware frame names agree.
Check the URDF, Ouster config, localisation config, preflight config, and
interface contracts before editing.
```

```text
Create a rover devcontainer workflow.
It must not store secrets in the image.
It should run doctor.py, contract checks, compile checks, and non-hardware tests.
Add a CI smoke test for the container build.
```

Weak tasks are broad and risky:

```text
Make autonomy work.
Clean up the repo.
Fix the rover.
```

Those are not impossible, but they invite guesswork at exactly the point where
the rover needs boring, inspectable steps.

## Hardware Rules

Keep these constraints explicit:

- real motion must fail closed;
- the safety stack and E-stop path are not optional for hardware work;
- the Jetson talks to the Teensy over USB serial for the current drivetrain path;
- the browser Gamepad bridge publishes `/cmd_vel_safe`, not direct motor commands;
- Foxglove should expose the runtime profile allowlist, not the full ROS graph;
- raw images and raw point clouds are not normal competition telemetry;
- private SSH keys, router passwords, Tailscale keys, and tokens must not be
  committed.

Remote support can cover code and docs, but someone in the lab must handle
plugs, power, E-stop tests, sensor placement, motor direction, and anything
involving physical risk.

## Review Bar

For code review, use the same bar as `AGENTS.md`:

- broken builds;
- runtime failures;
- unsafe robot behaviour;
- ROS topic, service, action, or TF contract mismatches;
- launch, config, deployment, or hardware bridge breakage;
- stale upstream API assumptions;
- missing tests for risky behaviour;
- divergence from local docs and contracts.

Do not spend review time on style nits unless they hide a real defect.

## Checks To Ask For

Cheap checks that are often worth running:

```bash
python3 .github/scripts/check_interface_contracts.py
python3 -m compileall -q tools scripts src/lunabot_bringup src/lunabot_drivetrain src/lunabot_safety src/lunabot_teleop
```

On a proper ROS 2 Humble machine:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
colcon test
colcon test-result --verbose
python3 tools/doctor.py --mode setup
```

On the Jetson, run runtime checks only after the relevant stack is actually up:

```bash
python3 tools/doctor.py --mode runtime
ros2 run lunabot_bringup runtime_profile check
```

If a local machine does not have ROS sourced, say that plainly in the result. Do
not turn a missing ROS install into a fake green check.
