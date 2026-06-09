#!/usr/bin/env python3
"""Xbox controller teleop over rosbridge WebSocket.

Reads the Xbox controller locally with pygame and publishes
geometry_msgs/Twist to the Jetson via roslibpy + rosbridge.

Usage:
    python teleop_roslibpy.py                      # default: Jetson Tailscale IP
    python teleop_roslibpy.py --host 10.10.52.121  # rover WiFi IP
    python teleop_roslibpy.py --host localhost      # SSH tunnel
"""

from __future__ import annotations

import argparse
import io
import math
import signal
import sys
import threading
import time

# Force UTF-8 output on Windows to avoid charmap encoding errors
if sys.platform == "win32":
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", errors="replace")

import pygame
import roslibpy

# ── Xbox controller mapping (SDL / pygame on Windows) ──────────────────
# Standard Xbox One / Xbox Series mapping via SDL2:
#   Axis 0: Left stick X   (left = -1, right = +1)
#   Axis 1: Left stick Y   (up = -1, down = +1)   ← inverted!
#   Axis 2: Right stick X  (left = -1, right = +1)
#   Axis 3: Right stick Y  (up = -1, down = +1)
#   Axis 4: Left trigger   (released = -1, full = +1)
#   Axis 5: Right trigger  (released = -1, full = +1)
#
# Match the rover's xbox_teleop.yaml:
#   axis_linear.x  = 1  → left stick Y  → forward/back
#   axis_angular.yaw = 2 → right stick X → turning
#   enable_button  = 9  → Start / Menu button (deadman)
#   enable_turbo   = 10 → Xbox / Guide button (turbo)

AXIS_LINEAR = 1       # Left stick Y
AXIS_ANGULAR = 2      # Right stick X
DEADMAN_BUTTON = 4    # LB / Left Bumper
ESTOP_BUTTON = 6      # Back button — publishes to /safety/estop
AXIS_LT        = 4   # Left trigger (released = -1, full = +1) — turbo

# Cytron MDD10A #1 actuator buttons — both channels move together (deadman must also be held)
# Y = both actuators extend, A = both actuators retract
BTN_ACT_EXTEND  = 3  # Y
BTN_ACT_RETRACT = 0  # A

# Deposition doors — X = extend (open), B = retract (close), deadman must be held
# Publishes to /deposition/actuator/cmd as Int8MultiArray [dir, dir]
BTN_ACT2_EXTEND  = 2  # X → extend doors  [1,  1]
BTN_ACT2_RETRACT = 1  # B → retract doors [-1, -1]

# BLDC (excavation) — deadman required
# RB (button 5) = clockwise at fixed test speed
# RT (axis 5)   = counter-clockwise, proportional to trigger depth
BLDC_CW_BUTTON = 5   # RB — clockwise
AXIS_RT        = 5   # Right trigger (released = -1, full = +1) — counter-clockwise

# Slew-rate limit: max speed change per loop tick.
# At 20 Hz, BLDC_RAMP_STEP=4 → ~1 s to reach full speed (80 units).
BLDC_RAMP_STEP = 4

# Speed limits matching xbox_teleop.yaml
SCALE_LINEAR = 0.35        # m/s normal
SCALE_LINEAR_TURBO = 0.6   # m/s turbo
SCALE_ANGULAR = 0.45       # rad/s normal
SCALE_ANGULAR_TURBO = 0.8  # rad/s turbo

DEADZONE = 0.08
PUBLISH_HZ = 20


def apply_deadzone(value: float, deadzone: float = DEADZONE) -> float:
    """Zero out small stick deflections."""
    if abs(value) < deadzone:
        return 0.0
    # Rescale so the output starts at 0 just past the deadzone
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def make_twist(linear_x: float, angular_z: float) -> dict:
    """Create a geometry_msgs/Twist dict for roslibpy."""
    return {
        "linear": {"x": linear_x, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": angular_z},
    }


def discover_deadman_button(js: pygame.joystick.JoystickType) -> int:
    """Interactive button discovery -- press the button you want as deadman."""
    print("\n>> Press the button you want to use as the DEADMAN (enable) switch...")
    print("   (This is the button you must HOLD to drive)\n")
    while True:
        pygame.event.get()  # Process events so joystick state updates
        for b in range(js.get_numbuttons()):
            if js.get_button(b):
                print(f"   [OK] Button {b} detected!")
                # Wait for release
                while js.get_button(b):
                    pygame.event.get()
                    time.sleep(0.05)
                return b
        time.sleep(0.05)


def discover_buttons_interactive(js: pygame.joystick.JoystickType) -> tuple[int, int | None]:
    """Walk the user through button mapping."""
    deadman = discover_deadman_button(js)

    print("\n>> Press a button for TURBO (hold for faster speed), or wait 5s to skip...")
    start = time.time()
    turbo = None
    while time.time() - start < 5.0:
        pygame.event.get()
        for b in range(js.get_numbuttons()):
            if js.get_button(b) and b != deadman:
                print(f"   [OK] Turbo button {b} detected!")
                while js.get_button(b):
                    pygame.event.get()
                    time.sleep(0.05)
                turbo = b
                break
        if turbo is not None:
            break
        time.sleep(0.05)

    if turbo is None:
        print("   Skipped turbo button.")

    return deadman, turbo


class TeleopController:
    """Reads Xbox controller and publishes Twist via rosbridge."""

    def __init__(
        self,
        ros: roslibpy.Ros,
        topic: str,
        joystick_id: int = 0,
        interactive_mapping: bool = False,
    ) -> None:
        self._ros = ros
        self._publisher = roslibpy.Topic(
            ros, topic, "geometry_msgs/Twist"
        )
        self._actuator_pub = roslibpy.Topic(
            ros, "/actuator/cmd", "std_msgs/Int8MultiArray"
        )
        self._deposition_pub = roslibpy.Topic(
            ros, "/deposition/actuator/cmd", "std_msgs/Int8MultiArray"
        )
        self._bldc_pub = roslibpy.Topic(
            ros, "/bldc/cmd", "std_msgs/Int8"
        )
        self._estop_pub = roslibpy.Topic(
            ros, "/safety/estop", "std_msgs/Bool"
        )
        self._running = False
        self._lock = threading.Lock()

        # Init pygame with a small window (required on Windows for input)
        pygame.init()
        pygame.joystick.init()
        # A small window is needed on Windows so pygame receives input events
        self._screen = pygame.display.set_mode((400, 200))
        pygame.display.set_caption("Lunabot Teleop - KEEP THIS WINDOW FOCUSED")
        self._font = pygame.font.SysFont("consolas", 14)
        self._render_status("Initialising...")

        count = pygame.joystick.get_count()
        if count == 0:
            raise RuntimeError("No controller found! Plug in your Xbox controller.")
        self._js = pygame.joystick.Joystick(joystick_id)
        self._js.init()
        print(f"[OK] Controller: {self._js.get_name()}")
        print(f"  Axes: {self._js.get_numaxes()}, Buttons: {self._js.get_numbuttons()}")

        # Button mapping
        if interactive_mapping:
            self._deadman, _ = discover_buttons_interactive(self._js)
        else:
            self._deadman = DEADMAN_BUTTON

        print(f"\n── Controls ──────────────────────────────")
        print(f"  Left stick Y  (axis {AXIS_LINEAR})  → forward / back")
        print(f"  Right stick X (axis {AXIS_ANGULAR})  → turn left / right")
        print(f"  Button {self._deadman}               → DEADMAN (hold to drive)")
        print(f"  LT (axis {AXIS_LT})             → TURBO (hold for speed)")
        print(f"  RB (button {BLDC_CW_BUTTON})               → BLDC clockwise (deadman required)")
        print(f"  RT (axis {AXIS_RT})             → BLDC counter-clockwise (deadman required)")
        print(f"  Back (button {ESTOP_BUTTON})           → E-STOP (publishes to /safety/estop)")
        print(f"  Ctrl+C                    → quit")
        print(f"──────────────────────────────────────────\n")

    def _render_status(self, line1: str, line2: str = "", line3: str = "") -> None:
        """Draw status text on the pygame window."""
        self._screen.fill((30, 30, 30))
        colors = {
            "STOPPED": (200, 60, 60),
            "NORMAL": (60, 200, 60),
            "TURBO": (220, 200, 40),
        }
        color = (180, 180, 180)
        for key, c in colors.items():
            if key in line1:
                color = c
                break
        y = 20
        for line in [line1, line2, line3]:
            if line:
                surf = self._font.render(line, True, color)
                self._screen.blit(surf, (15, y))
                y += 30
                color = (160, 160, 160)  # dimmer for subsequent lines
        # Reminder at bottom
        hint = self._font.render("Keep this window focused for input!", True, (100, 100, 100))
        self._screen.blit(hint, (15, 160))
        pygame.display.flip()

    def start(self) -> None:
        """Start the publish loop."""
        self._running = True
        self._publisher.advertise()
        self._actuator_pub.advertise()
        self._deposition_pub.advertise()
        self._bldc_pub.advertise()
        self._estop_pub.advertise()
        self._last_actuator   = [0, 0]
        self._last_deposition = [0, 0]
        self._last_bldc       = 0
        self._bldc_current    = 0   # ramped output, stepped towards target each tick
        self._prev_estop_btn  = False
        period = 1.0 / PUBLISH_HZ

        print("[ACTIVE] Teleop running -- hold deadman button and move sticks to drive")
        print("         Keep the pygame window focused for controller input!\n")

        try:
            while self._running and self._ros.is_connected:
                t_loop_start = time.monotonic()

                # Handle pygame window events (close button, etc.)
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self._running = False
                        break

                twist, status = self._read_controller()
                self._publisher.publish(roslibpy.Message(twist))

                # Cytron #1 actuator buttons (Y/A) — deadman must be held
                deadman_held = self._js.get_button(self._deadman)
                act_dir = 0
                if deadman_held:
                    if self._js.get_button(BTN_ACT_EXTEND):
                        act_dir = 1
                    elif self._js.get_button(BTN_ACT_RETRACT):
                        act_dir = -1
                if [act_dir, act_dir] != self._last_actuator:
                    self._actuator_pub.publish(roslibpy.Message({
                        "layout": {"dim": [], "data_offset": 0},
                        "data": [act_dir, act_dir],
                    }))
                    self._last_actuator = [act_dir, act_dir]

                # Deposition doors (X/B) — deadman must be held
                dep_dir = 0
                if deadman_held:
                    if self._js.get_button(BTN_ACT2_EXTEND):
                        dep_dir = 1
                    elif self._js.get_button(BTN_ACT2_RETRACT):
                        dep_dir = -1
                if [dep_dir, dep_dir] != self._last_deposition:
                    self._deposition_pub.publish(roslibpy.Message({
                        "layout": {"dim": [], "data_offset": 0},
                        "data": [dep_dir, dep_dir],
                    }))
                    self._last_deposition = [dep_dir, dep_dir]

                # BLDC — deadman must be held; RB = clockwise, RT = counter-clockwise.
                # Target speed is computed from inputs; actual output ramps towards it
                # at BLDC_RAMP_STEP units/tick to prevent mechanical shock on start/stop.
                # Published every tick (not just on change) to keep the firmware's
                # 500 ms watchdog fed reliably.
                bldc_target = 0
                if deadman_held:
                    rb_held = self._js.get_button(BLDC_CW_BUTTON)
                    rt_val = (self._js.get_axis(AXIS_RT) + 1.0) / 2.0  # 0..1
                    if rb_held:
                        bldc_target = 80
                    elif rt_val > 0.05:
                        bldc_target = -int(round(rt_val * 127))
                bldc_target = max(-127, min(127, bldc_target))
                if self._bldc_current < bldc_target:
                    self._bldc_current = min(bldc_target, self._bldc_current + BLDC_RAMP_STEP)
                elif self._bldc_current > bldc_target:
                    self._bldc_current = max(bldc_target, self._bldc_current - BLDC_RAMP_STEP)
                self._bldc_pub.publish(roslibpy.Message({"data": self._bldc_current}))
                self._last_bldc = self._bldc_current

                # E-stop — Back button, edge-triggered: publish True on press.
                # Release / restart is handled by the Jetson safety stack (U + R commands).
                estop_btn = self._js.get_button(ESTOP_BUTTON)
                if estop_btn and not self._prev_estop_btn:
                    self._estop_pub.publish(roslibpy.Message({"data": True}))
                    print("\n[E-STOP] Published to /safety/estop — motors latched off.")
                self._prev_estop_btn = estop_btn

                # Update the pygame window with current status
                lin = twist["linear"]["x"]
                ang = twist["angular"]["z"]
                self._render_status(
                    status,
                    f"Publishing: lin={lin:+.2f} m/s  ang={ang:+.2f} rad/s",
                    f"Topic: {self._publisher.name}",
                )

                # Sleep only for the remaining time in this period so the loop
                # runs at a steady PUBLISH_HZ regardless of processing overhead.
                # This keeps the BLDC watchdog (500 ms) fed reliably.
                elapsed = time.monotonic() - t_loop_start
                time.sleep(max(0.0, period - elapsed))
        except KeyboardInterrupt:
            pass
        finally:
            # Send zero on exit
            self._publisher.publish(roslibpy.Message(make_twist(0.0, 0.0)))
            self._actuator_pub.publish(roslibpy.Message({
                "layout": {"dim": [], "data_offset": 0},
                "data": [0, 0],
            }))
            self._deposition_pub.publish(roslibpy.Message({
                "layout": {"dim": [], "data_offset": 0},
                "data": [0, 0],
            }))
            self._bldc_current = 0
            self._bldc_pub.publish(roslibpy.Message({"data": 0}))
            time.sleep(0.05)
            self._publisher.unadvertise()
            self._actuator_pub.unadvertise()
            self._deposition_pub.unadvertise()
            self._bldc_pub.unadvertise()
            self._estop_pub.unadvertise()
            print("\n[STOPPED] Teleop stopped -- zero velocity sent.")

    def stop(self) -> None:
        """Signal the publish loop to stop."""
        self._running = False

    def _read_controller(self) -> tuple[dict, str]:
        """Read current controller state and return (twist_dict, status_string)."""
        deadman_held = self._js.get_button(self._deadman)
        # LT axis: released = -1, full press = +1 — turbo when pressed past 20%
        turbo_held = self._js.get_axis(AXIS_LT) > -0.6

        if not deadman_held:
            return make_twist(0.0, 0.0), "STOPPED (press deadman to drive)"

        # Read sticks
        raw_linear = -self._js.get_axis(AXIS_LINEAR)  # Invert: up = forward
        raw_angular = -self._js.get_axis(AXIS_ANGULAR)  # Invert: left = positive yaw

        linear = apply_deadzone(raw_linear)
        angular = apply_deadzone(raw_angular)

        # Apply speed scaling
        if turbo_held:
            linear_vel = linear * SCALE_LINEAR_TURBO
            angular_vel = angular * SCALE_ANGULAR_TURBO
            mode = "TURBO"
        else:
            linear_vel = linear * SCALE_LINEAR
            angular_vel = angular * SCALE_ANGULAR
            mode = "NORMAL"

        twist = make_twist(linear_vel, angular_vel)
        status = (
            f"{mode}  lin={linear_vel:+.2f} m/s  ang={angular_vel:+.2f} rad/s  "
            f"stick=({raw_linear:+.2f}, {raw_angular:+.2f})"
        )
        return twist, status


def main() -> None:
    """Entry point."""
    parser = argparse.ArgumentParser(
        description="Xbox controller teleop via rosbridge WebSocket"
    )
    parser.add_argument(
        "--host",
        default="100.71.241.9",
        help="Jetson rosbridge host (default: Tailscale IP 100.71.241.9)",
    )
    parser.add_argument(
        "--port", type=int, default=9090,
        help="rosbridge WebSocket port (default: 9090)",
    )
    parser.add_argument(
        "--topic",
        default="/cmd_vel_safe",
        help="ROS topic to publish Twist on (default: /cmd_vel_safe)",
    )
    parser.add_argument(
        "--joy-id", type=int, default=0,
        help="Joystick device index (default: 0)",
    )
    parser.add_argument(
        "--map-buttons", action="store_true",
        help="Interactive button mapping on startup",
    )
    args = parser.parse_args()

    print(f"Connecting to rosbridge at ws://{args.host}:{args.port} ...")
    ros = roslibpy.Ros(host=args.host, port=args.port)

    try:
        ros.run()
    except Exception as exc:
        print(f"\n[ERROR] Could not connect to rosbridge at {args.host}:{args.port}")
        print(f"   Error: {exc}")
        print(f"\n   Make sure rosbridge is running on the Jetson:")
        print(f"   ros2 launch rosbridge_server rosbridge_websocket_launch.xml")
        sys.exit(1)

    print(f"[OK] Connected to rosbridge at {args.host}:{args.port}")
    print(f"  Publishing to: {args.topic}\n")

    controller = TeleopController(
        ros,
        args.topic,
        joystick_id=args.joy_id,
        interactive_mapping=args.map_buttons,
    )

    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        controller.stop()

    signal.signal(signal.SIGINT, signal_handler)

    try:
        controller.start()
    finally:
        ros.terminate()
        pygame.quit()


if __name__ == "__main__":
    main()
