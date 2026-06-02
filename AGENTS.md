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
- `skills/README.md`
- package READMEs under `src/*/README.md`
- the GitHub wiki when it is checked out at `wiki/`

When a change touches a ROS interface, TF frame, action, topic, launch file, parameter file, hardware bridge or simulation bridge, check the relevant local contract and docs before recommending a fix.

## Local Robotics Skills

Use the repo-local robotics skills in `skills/*/SKILL.md` before planning or editing related work. If your tool auto-discovers project skills, it should load them from there. Otherwise, read the relevant `SKILL.md` file directly.

- Use `skills/ros2/SKILL.md` for ROS 2 Humble nodes, launch files, parameters, QoS, actions, lifecycle nodes, Nav2, DDS, `colcon`, `ament`, `package.xml`, `setup.py`, `CMakeLists.txt`, rosdep and workspace overlay work.
- Use `skills/docker-ros2-development/SKILL.md` for Docker, devcontainers, ROS 2 CI images, DDS networking in containers, GPU passthrough and USB/device passthrough.
- Use `skills/robot-bringup/SKILL.md` for Jetson bring-up, systemd services, launch composition, startup ordering, udev rules, watchdogs, log rotation and long-running robot services.
- Use `skills/robotics-design-patterns/SKILL.md` for rover architecture, behaviour trees or state machines, watchdogs, heartbeats, fail-closed behaviour, graceful degradation, hardware abstraction layers and sim-to-real decisions.
- Use `skills/robotics-software-principles/SKILL.md` for robotics code reviews, module boundaries, hardware interfaces, dependency direction, configuration management, error handling and refactors that could affect robot behaviour.
- Use `skills/robotics-testing/SKILL.md` for unit tests, launch tests, integration tests, mock hardware, deterministic replay, CI checks, hardware-in-the-loop plans and regression tests for safety or runtime behaviour.
- Use `skills/robot-perception/SKILL.md` for OAK-D, camera calibration, depth topics, AprilTags, Ouster/LiDAR, point clouds, TF/extrinsics, sensor timing, perception latency and sensor bring-up.
- Use `skills/robotics-security/SKILL.md` for SSH, router/network hardening, secrets, certificates, Docker/devcontainer security, ROS 2/DDS security, E-stop security and competition network exposure.
- Use `skills/ros2-web-integration/SKILL.md` for browser Gamepad, Foxglove-adjacent web bridges, WebSockets, REST wrappers, MJPEG/WebRTC and web control surface security.

Do not use the upstream ROS 1 skill for this repo.
