# Innex1 Rover - CodeRabbit Review Guidelines

Project:
University of Leicester Lunabotics 2026 lunar rover.

## Stack

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Ignition Fortress
- C++ / Python

## Documentation

Architecture and contracts (our source of truth): [https://github.com/KJdotIO/innex1-rover/wiki](https://github.com/KJdotIO/innex1-rover/wiki)

Review PRs for consistency with documented interfaces, but note that contracts may evolve - code can lead, docs can follow.

## Standards

- ROS 2 conventions: snake_case topics/nodes, CamelCase message types
- Python: PEP 8, docstrings on public functions
- C++: ROS 2 style guide
- `colcon build` and `colcon test` should pass cleanly
- Appropriate QoS settings

## Docstring Guidelines

- Use the **title + description** format:

  ```python
  """
  Short title line (what it does).

  Brief description or context (1-2 sentences max).
  """
  ```

- Each line must be under 99 characters (PEP 8 / flake8 E501)
- Do NOT generate verbose multi-paragraph docstrings
- Avoid restating parameter names or return types that are obvious from the code
- Launch files: title = "Generate a launch description for X", description = what nodes/includes

## Focus Areas

- TF frame consistency
- QoS compatibility between pub/sub
- Error handling for sensor failures
- Sim/hardware abstraction (same interfaces, different drivers)

Be direct and helpful. Flag genuine issues - bugs, convention violations, QoS mismatches - but don't nitpick stylistic preferences that don't affect functionality. If something works and follows the documented conventions, it's fine to merge.

## Review Tone

- **Pragmatic over pedantic** - if CI passes and the code works, it's good to merge
- Focus on bugs, logic errors, and convention violations that actually matter
- Limit suggestions to what's blocking or genuinely important
- If you're unsure whether something is worth flagging, it probably isn't
- Don't suggest rewrites of working code for marginal improvements

## Not Worth Flagging

- Minor whitespace/formatting (let the linters handle it)
- Personal style preferences that don't violate PEP 8 or ROS 2 conventions
- "Could be slightly more elegant" suggestions on working code
- Equivalent syntax alternatives (if both work the same, don't suggest switching)
- "Nice to have" config that isn't causing actual issues
