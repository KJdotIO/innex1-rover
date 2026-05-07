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
