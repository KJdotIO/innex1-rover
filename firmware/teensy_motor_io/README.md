# Teensy motor IO firmware

This is the host-testable core for the Teensy 4.1 motor IO controller.

The Teensy is treated as a hard real-time IO adapter, not as a ROS node. The
Jetson sends framed USB CDC serial commands; the Teensy applies local safety
checks, drives the motor controllers, counts encoders, and reports telemetry
back to ROS through the Jetson bridge.

## Design rules

- All command frames are versioned, COBS-framed, and protected by CRC-16.
- Bad frames are ignored. They must not change outputs.
- If the Jetson heartbeat goes stale, motor outputs go to zero.
- E-stop, motion inhibit, controller faults, and an unconfirmed current-limit
  configuration force outputs to zero.
- Pin assignments live in the Teensy hardware adapter, not in the protocol or
  safety core.

## Host tests

These tests run without Arduino tooling:

```bash
cmake -S firmware/teensy_motor_io -B build/teensy_motor_io
cmake --build build/teensy_motor_io
ctest --test-dir build/teensy_motor_io --output-on-failure
```

The Arduino/Teensy sketch should stay thin: serial read/write, GPIO/PWM setup,
encoder ISR wiring, and calls into this core.
