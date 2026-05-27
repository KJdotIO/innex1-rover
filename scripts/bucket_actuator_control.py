#!/usr/bin/env python3
 
import argparse
from enum import Enum
import time
import serial
 

class BucketActuatorState(Enum):
    """Operator-facing bucket actuator states."""

    BUCKET_DOWN = "bucket down"
    WAITING = "waiting"
    BUCKET_UP = "bucket up"
    STOPPED = "stopped"


STATE_BY_UART_TEXT = {
    state.value: state
    for state in BucketActuatorState
}


def print_state(state):
    print(f"Bucket actuator state: {state.value}")


def parse_uart_state(line):
    return STATE_BY_UART_TEXT.get(line.strip().lower())

 
def terminal_state_for_command(command):
    if command == "s":
        return BucketActuatorState.STOPPED
    return None


def bucket_sequence_complete(command, state, seen_bucket_down, seen_waiting):
    if command == "g":
        return (
            state == BucketActuatorState.BUCKET_UP
            and seen_bucket_down
            and seen_waiting
        )

    return state == terminal_state_for_command(command)


def send_command(port, baud, command, listen_timeout_s):
    with serial.Serial(port, baudrate=baud, timeout=1) as ser:
        time.sleep(2)
        ser.write(command.encode("ascii"))
        ser.flush()
 
        print(f"Sent command: {command}")
 
        seen_bucket_down = False
        seen_waiting = False
        start_time = time.time()
        while time.time() - start_time < listen_timeout_s:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                state = parse_uart_state(line)
                if state is None:
                    print(line)
                else:
                    print_state(state)
                    if state == BucketActuatorState.BUCKET_DOWN:
                        seen_bucket_down = True
                    elif state == BucketActuatorState.WAITING:
                        seen_waiting = True

                    if bucket_sequence_complete(
                        command,
                        state,
                        seen_bucket_down,
                        seen_waiting,
                    ):
                        break
 
 
def main():
    parser = argparse.ArgumentParser(
        description="Control bucket actuators through the Nucleo serial port"
    )
 
    parser.add_argument(
        "command",
        choices=["g", "s"],
        help="g = run actuator sequence, s = stop immediately",
    )
 
    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="Nucleo serial port, default /dev/ttyACM0",
    )
 
    parser.add_argument(
        "--baud",
        type=int,
        default=9600,
        help="Serial baud rate, default 9600",
    )

    parser.add_argument(
        "--listen-timeout-s",
        type=float,
        default=60.0,
        help="Seconds to listen for Nucleo state updates, default 60",
    )
 
    args = parser.parse_args()
    send_command(args.port, args.baud, args.command, args.listen_timeout_s)
 
 
if __name__ == "__main__":
    main()
