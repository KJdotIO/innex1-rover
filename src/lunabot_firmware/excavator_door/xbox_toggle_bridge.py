import argparse
import sys
import time

import pygame
import serial


XBOX_B_BUTTON_INDEX = 1


def open_first_controller():
    pygame.joystick.init()
    if pygame.joystick.get_count() < 1:
        raise RuntimeError("No controller detected")

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    return joystick


def main():
    parser = argparse.ArgumentParser(
        description="Send a toggle command to the STM32 when Xbox B is pressed."
    )
    parser.add_argument("--port", required=True, help="Serial port, for example COM3")
    parser.add_argument("--baud", type=int, default=9600, help="Serial baud rate")
    args = parser.parse_args()

    pygame.init()

    try:
        joystick = open_first_controller()
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as exc:
        print(f"Startup failed: {exc}", file=sys.stderr)
        pygame.quit()
        return 1

    print(f"Connected to controller: {joystick.get_name()}")
    print(f"Connected to serial port: {args.port} @ {args.baud}")
    print("Press Xbox B to toggle the door. Press Ctrl+C to exit.")

    b_was_pressed = False

    try:
        while True:
            pygame.event.pump()

            b_pressed = bool(joystick.get_button(XBOX_B_BUTTON_INDEX))
            if b_pressed and not b_was_pressed:
                ser.write(b"b")
                ser.flush()
                print("Sent toggle command")

            b_was_pressed = b_pressed
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nExiting")
    finally:
        ser.close()
        pygame.quit()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
