# Hardware Week Runbook

Use this during the first week back with the rover. It gives the team one
place for the commands, checks, callouts, and evidence capture steps needed for
hardware bring-up and scored-run rehearsal.

The rulebook constraints that shape this are simple:

- setup is 10 minutes;
- the run is 20 minutes;
- the rover has to move within the first 5 minutes of the run;
- autonomy must be announced before it starts, and declared complete or failed
  before anyone touches the controls again;
- average comms use must stay at or below 4,000 Kbps;
- only mission-critical electronics come into Mission Control.

## Before You Leave The Pit

Run these checks before the rover is in the arena.

On the Jetson:

```bash
cd ~/innex1-rover
git status --short
git branch --show-current
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to lunabot_bringup
source install/setup.bash
```

If you are testing a PR, use the real Jetson checkout:

```bash
cd ~/innex1-rover
git fetch origin
git checkout main
git pull --ff-only origin main
git checkout <branch-name>
colcon build --symlink-install --packages-up-to lunabot_bringup
source install/setup.bash
```

If the checkout is dirty, stop and read the diff. Do not blindly reset the
Jetson. Someone may have changed a local port, serial device, or Zed setting for
a reason.

## Kill The Old Run

Before each fresh launch, clear old ROS, Nav2, RViz, and Gazebo processes:

```bash
pkill -f "ros2 launch" || true
pkill -f "ros2 run" || true
pkill -f "rviz2" || true
pkill -f "ign gazebo" || true
pkill -f "gz sim" || true
sleep 2
ps -eo pid,cmd | egrep "ros2|rviz2|ign gazebo|gz sim|mission_manager|nav2" | grep -v egrep || true
```

That last command should not show any active rover processes. If something is
still running, stop it before launching a fresh stack.

## Start Lean

For a competition-style software posture, check the runtime profile:

```bash
ros2 run lunabot_bringup runtime_profile check
ros2 run lunabot_bringup runtime_profile show --profile hardware_competition
```

For Mission Control, keep Foxglove limited to the topics needed to operate the
rover. Show mission state, safety, drivetrain status, excavation status,
localisation status, diagnostics, and the minimum camera view needed for the
run. Do not stream raw point clouds or every debug costmap during a scored run
unless the team has a specific reason to spend the bandwidth.

The useful telemetry set is:

```text
/mission/state
/mission/autonomy_mode
/mission/time_remaining_s
/mission/cycle_count
/mission/last_failure_reason
/safety/estop
/safety/motion_inhibit
/drivetrain/status
/excavation/status
/localisation/start_zone_status
/diagnostics
/power/telemetry
```

Check bandwidth at the network edge as well as in ROS:

```bash
ip -s link
sar -n DEV 1 10
```

If `sar` is not installed, use the router dashboard, `ifstat`, or `nload`. The
limit applies to the robot comms link, not to an individual ROS topic.

## First Motion

Do not start with full autonomy. Start with something you can stop.

For drivetrain bench work, use the Sabertooth bring-up guide:

```bash
ros2 launch lunabot_drivetrain drivetrain_bench.launch.py max_throttle:=0.2
```

Then send a low-speed command from another terminal:

```bash
ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"
ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Watch these while doing it:

```bash
ros2 topic echo /drivetrain/status
ros2 topic echo /drivetrain/telemetry
ros2 topic echo /power/telemetry
ros2 topic echo /safety/motion_inhibit
```

If the rover is on the ground, agree the stop command and operator before any
motion test starts.

After an E-stop event, releasing the physical E-stop does not re-enable motion
by itself. Confirm `/safety/estop` is false, then clear the software inhibit:

```bash
ros2 topic echo /safety/estop --once
ros2 topic pub --once /safety/reset_motion_inhibit std_msgs/msg/Bool "{data: true}"
ros2 topic echo /safety/motion_inhibit --once
```

Only continue when `/safety/motion_inhibit` reports `data: false`.

For early runs without a wired battery sensor, publish the value from the
visible power meter manually:

```bash
ros2 run lunabot_bringup manual_power_telemetry \
  --ros-args \
  -p profile:=lipo_6s \
  -p bus_voltage_v:=22.2 \
  -p bus_current_a:=0.0 \
  -p energy_wh:=0.0
```

Use `lipo_4s` if the rover is on a 4S pack. The default voltage is unavailable,
so diagnostics will say power telemetry is missing until someone enters a real
value.

## Before Autonomy

The minimum useful checks are:

```bash
ros2 topic hz /diagnostics
ros2 topic echo /mission/state --once
ros2 topic echo /mission/autonomy_mode --once
ros2 topic echo /localisation/start_zone_status --once
ros2 action list | grep navigate
ros2 lifecycle nodes
```

In RViz or Foxglove, check:

- TF is coherent: `map`, `odom`, `base_footprint`, `base_link`;
- the rover pose is believable;
- costmaps are not filled solid around the rover;
- `/cmd_vel_safe` changes when Nav2 or teleop commands motion;
- `/cmd_vel_gated` only opens when drivetrain and safety state allow it.

For autonomy scoring, use explicit callouts:

1. Tell the Mission Control Judge what autonomy attempt is about to begin.
2. Start the attempt.
3. Do not touch the controller or laptop while the rover is still in that
   autonomy attempt.
4. Announce whether the attempt completed or failed before taking manual
   control back.

Do this every time. The judge needs to know when autonomy starts, when it ends,
and whether manual control has resumed.

## Evidence Bags

Record bags for runs that need later review. Do not keep every messy half-run
as "golden"; use that label only for runs clean enough to trust later.

For a lean evidence pack around a command:

```bash
ros2 run lunabot_bringup mission_evidence \
  --profile minimal \
  --label hardware-check \
  -- ros2 launch lunabot_bringup mission_manager.launch.py
```

Evidence packs live under:

```text
~/innex1_mission_evidence/
```

Before calling a bag useful, check:

```bash
ros2 bag info ~/innex1_mission_evidence/<pack>/bag
cat ~/innex1_mission_evidence/<pack>/manifest.json
```

The manifest should tell you the command, profile, git SHA, return codes, and
summary. If the summary says failed, keep it as a fault bag and name it that way.

## When Something Fails

Use this order. It checks the high-level state before moving into subsystem
debugging.

First, check the summary topics:

```bash
ros2 topic echo /mission/state --once
ros2 topic echo /mission/autonomy_mode --once
ros2 topic echo /mission/last_failure_reason --once
ros2 topic echo /diagnostics --once
ros2 topic echo /power/telemetry --once
```

If the rover does not move:

- check `/safety/estop` and `/safety/motion_inhibit`;
- if E-stop was used, reset `/safety/motion_inhibit` after `/safety/estop`
  is false;
- check `/drivetrain/status`;
- check whether `/cmd_vel_safe` is being produced;
- check whether `/cmd_vel_gated` is blocked;
- only then look at motors, serial, and Sabertooth wiring.

If navigation fails:

- check localisation freshness first;
- check TF before touching planner parameters;
- check whether Nav2 lifecycle nodes are active;
- check whether costmaps are empty, stale, or inflated into a wall;
- check the mission log for the first failed attempt, not the last noisy
  recovery.

If excavation or deposition fails:

- stop motion first;
- check the mechanism status topic;
- check driver fault and limit switch state;
- record the bag and log before cycling power.

If comms degrade:

- stop raw streams;
- close RViz if Foxglove has enough telemetry;
- check router counters;
- keep the rover responsive before preserving high-bandwidth visualisation.

## End Of Run

When the run ends, stop the rover first:

```bash
ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Do not disconnect the robot or pack Mission Control until the judge says so.
The rulebook allows the robot to need relocation or unloading after the timer.

Then save the evidence path, branch, and short note:

```text
Bag:
Branch:
What happened:
What to try next:
```

Keep the note short. Record what happened and what the next test should check.
