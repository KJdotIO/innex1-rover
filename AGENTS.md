# Repository Instructions

Use British English.

## Review Guidance

This is a ROS 2 Humble rover stack for UK Lunabotics. Reviews should focus on correctness, safety and integration risk.

Only report issues that could cause broken builds, runtime failures, unsafe robot behaviour, ROS 2 topic/service/action or TF contract mismatches, launch/config/deployment breakage, stale upstream API assumptions, missing tests for risky behaviour, or divergence from the project wiki and architecture docs.

Do not comment on style, naming, formatting, speculative refactors, or broad rewrites unless they hide a concrete defect.

Treat these as important local sources of truth:

- `README.md`
- `ROVER_CODING_STANDARD.md`
- `.github/contracts/interface_contracts.json`
- package READMEs under `src/*/README.md`
- the GitHub wiki when it is checked out at `wiki/`

When a change touches a ROS interface, TF frame, action, topic, launch file, parameter file, hardware bridge or simulation bridge, check the relevant local contract and docs before recommending a fix.

## Local Robotics Skills

If local robotics skills are available, use the relevant skill before planning or editing. These skills are not vendored into the repo, so treat them as optional help rather than a hard dependency.

- Use `ros2-development` for ROS 2 Humble nodes, launch files, parameters, QoS, actions, lifecycle nodes, Nav2, DDS, `colcon`, `ament`, `package.xml`, `setup.py`, `CMakeLists.txt`, rosdep and workspace overlay work.
- Use `robotics-design-patterns` for rover architecture, behaviour trees or state machines, watchdogs, heartbeats, fail-closed behaviour, graceful degradation, hardware abstraction layers and sim-to-real decisions.
- Use `robotics-software-principles` for robotics code reviews, module boundaries, hardware interfaces, dependency direction, configuration management, error handling and refactors that could affect robot behaviour.
- Use `robotics-testing` for unit tests, launch tests, integration tests, mock hardware, deterministic replay, CI checks, hardware-in-the-loop plans and regression tests for safety or runtime behaviour.
- Use `robot-perception` for OAK-D, camera calibration, depth topics, AprilTags, Ouster/LiDAR, point clouds, TF/extrinsics, sensor timing, perception latency and sensor bring-up.
- Use `robotics-security` for SSH, router/network hardening, secrets, certificates, Docker/devcontainer security, ROS 2/DDS security, E-stop security and competition network exposure.

If a skill is unavailable, continue with the repo docs and say that the local skill was not available. Do not invent skill output.
