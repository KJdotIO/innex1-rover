# Competition Remote Support

Use this for the remote support period before competition. The target is to have
the rover ready for competition use by 2026-06-15, with the competition on
2026-06-16 and 2026-06-17.

Assume the Friday 2026-05-29 lab checklist was not completed unless there is
fresh evidence in Linear, a dated note, a rosbag, or a commit that says it was.

## Working Model

Remote work can cover code, docs, Linear, GitHub, simulation, review, and Jetson
work over SSH or Tailscale.

Lab work is still needed for anything involving plugs, power cycling, router
screens, motor direction, E-stop wiring, camera placement, AprilTag movement,
LiDAR mounting, USB swaps, fuse checks, and anything where someone needs to see,
hear, or safely restrain the rover.

Use this rule when planning a task:

```text
Can it be done with the repo, Linear, GitHub, simulation, or a shell on the Jetson?
  yes -> remote-friendly
  no  -> assign a lab owner and write the physical steps plainly
```

## Private Access Details

Do not commit passwords, private keys, Tailscale auth keys, or router admin
credentials to this repo.

Keep the actual values in the team's private channel:

```text
Jetson SSH user:
Jetson Tailscale IP:
Jetson rover-LAN IP:
Router SSID:
Router admin URL:
Router admin owner:
Foxglove URL:
Browser Gamepad URL:
Current Git branch on Jetson:
Known-good commit:
```

The current bench docs and config use `192.168.8.20` for the Jetson on the rover
LAN. If the final competition IP changes, update `docs/hardware/gl-a1300-router.md`,
`docs/foxglove/README.md`, and
`src/lunabot_bringup/config/ouster_lidar_debug.yaml` together.

Known 2026-05-22 Slate Plus lab evidence:

```text
Jetson wired LAN: 192.168.8.20
Slate Plus:       192.168.8.1
Ouster OS1-128:   192.168.8.176
```

Mac on Slate Wi-Fi could ping the Jetson and Ouster with 0 per cent packet loss.
SSH worked from the Mac to the Jetson over Slate using the private
`innex1-slate` alias. The normal Foxglove bridge was reachable on `8765`.

Known 2026-05-28 Wi-Fi control evidence: a Mac browser reached the Jetson HTTPS
Gamepad bridge at `https://192.168.8.20:9443`, and the Jetson published into
`/cmd_vel_safe` so the velocity gate remained in the command path.

## Cold Start

From the lab side:

1. Power the router and wait for the rover SSID.
2. Power the Jetson and confirm it joins the rover LAN or Tailscale.
3. Power the Ouster only when its mount, cable strain relief, and 12 V supply are
   safe. Wait at least 60 seconds before expecting point clouds.
4. Keep the rover lifted, restrained, or disabled before drivetrain tests.
5. Confirm who owns the stop action before any motion command.

From the remote side, once SSH works:

```bash
cd ~/innex1-rover
git status --short
git branch --show-current
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to lunabot_bringup
source install/setup.bash
python3 tools/doctor.py --mode setup
```

If the Jetson checkout is dirty, read the diff before touching it. Local changes
may be port names, device IDs, or one-off hardware notes.

## First Motion

Do not start with full autonomy.

Until the safety bring-up ticket is fixed, explicitly start the safety stack
before drivetrain testing and prove the E-stop story before trusting motion:

```bash
ros2 launch lunabot_safety safety.launch.py
```

The parameter backend is useful for software tests, but it is not a physical
E-stop. Hardware motion must not be treated as competition-ready until the real
input path, pin, polarity, latch, and reset behaviour have been demonstrated.

For drivetrain bench work:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py max_throttle:=0.2
```

Watch the safety and drivetrain topics while the lab owner performs the test:

```bash
ros2 topic echo /safety/estop --once
ros2 topic echo /safety/motion_inhibit --once
ros2 topic echo /drivetrain/status
ros2 topic echo /drivetrain/telemetry
```

Use one low-speed command first:

```bash
ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"
ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Operator View

The normal operator view is Foxglove connected to the Jetson bridge:

```text
ws://<jetson-ip>:8765
```

Start it on the Jetson with the lean competition profile:

```bash
ros2 launch lunabot_bringup foxglove_ground_control.launch.py \
  profile:=hardware_competition \
  use_sim_time:=false
```

Use the layout in `docs/foxglove/mission_control.layout.json`.

Do not stream raw camera images, raw depth, raw point clouds, or heavy debug
costmaps during a scored run unless the team has consciously chosen to spend the
bandwidth.

## Browser Gamepad Bridge

For operator-laptop driving over the rover network, use the Jetson-hosted browser
Gamepad bridge. It publishes into `/cmd_vel_safe`, so the normal velocity gate,
drivetrain timeout, and Teensy watchdog still apply.

Use HTTPS on the rover LAN:

```bash
openssl req -x509 -newkey rsa:2048 -nodes \
  -keyout /tmp/lunabot_web_gamepad.key \
  -out /tmp/lunabot_web_gamepad.crt \
  -days 7 \
  -subj "/CN=innex1-web-gamepad" \
  -addext "subjectAltName=IP:<jetson-rover-lan-ip>,DNS:innex1-desktop"

ros2 launch lunabot_teleop web_gamepad_bridge.launch.py \
  bind_host:=0.0.0.0 \
  port:=9443 \
  tls_cert_file:=/tmp/lunabot_web_gamepad.crt \
  tls_key_file:=/tmp/lunabot_web_gamepad.key
```

Then open:

```text
https://<jetson-rover-lan-ip>:9443
```

The certificate SAN IP must match the address the operator will browse to, or
some browsers will reject the page before the Gamepad API can be used. Accept
the self-signed certificate warning only for the rover address. Do not expose
this bridge on an untrusted network.

## Sensor Work Still Needs Hands

The remaining lab-side work is not glamorous, which is normally how you know it
matters.

High-value physical checks:

- mount the Ouster, confirm orientation, cable strain relief, and the final TF;
- smoke-test Ouster through the router using
  `src/lunabot_bringup/config/ouster_lidar_debug.yaml` as the current config;
  the historical low-rate room baseline is recorded in
  `docs/ouster_lidar_room_debug.md`;
- mount the OAK-D, confirm the USB-C path, depth topics, and AprilTag view;
- prove the real physical E-stop input and reset sequence;
- prove the real power telemetry source, or explicitly document that the manual
  publisher is only a temporary operator aid;
- confirm router SSID, channel, IPs, and bandwidth checks;
- record at least one useful evidence pack from a controlled hardware run.

## Docker Track

There is a ROS CI image and a lightweight rover devcontainer.
See `docs/docker_workflow.md` for the current image policy and local commands.

Do not rebuild a web dashboard path for competition. The useful Docker target is
not "run the whole rover on a laptop". The safer target is:

- a devcontainer or rover-dev image for contributors who need a repeatable
  Ubuntu/ROS tooling shell;
- helper scripts for running `doctor.py`, contract checks, compile checks, and
  non-hardware tests;
- clear network variables for the Jetson bridge and Foxglove URL;
- no secrets baked into images;
- CI smoke tests that build the image and run the cheap repo checks.

Build images on pull requests for smoke testing, but publish only trusted tags:
`main` for the latest main-branch dev image, `sha-<commit>` for immutable
debugging, and release tags only when the team deliberately cuts a release.

## Linear Board Shape

Use a 2026-06-15 milestone with these tracks:

- safety and fail-closed behaviour;
- drivetrain and Teensy validation;
- router, SSH, Tailscale and operator access;
- Docker/devcontainer workflow;
- OAK-D and AprilTag validation;
- Ouster mounting, TF and legal LiDAR path;
- power telemetry and inspection evidence;
- final docs, review guidance, and runbook rehearsal.

Every ticket should say whether it is remote-friendly or lab-required. If a
ticket needs hands in the lab, name the physical action in the first paragraph.

## Coding Assistance

Coding assistants are useful here only when they are given the rover's real
constraints. Start with:

- `AGENTS.md`;
- `README.md`;
- `ROVER_CODING_STANDARD.md`;
- `.github/contracts/interface_contracts.json`;
- `skills/README.md`;
- `docs/active_runtime_paths.md`;
- `docs/hardware_week_runbook.md`;
- package READMEs under `src/*/README.md`;
- any checked-out wiki pages, if present.

Before accepting a change that touches topics, TF, launch files, hardware
bridges, safety, power, or operator command paths, check the contracts, run the
relevant tests, and make sure the change fails closed on real hardware.
