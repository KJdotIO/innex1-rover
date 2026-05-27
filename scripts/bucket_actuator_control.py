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

 
def send_command(port, baud, command):
    with serial.Serial(port, baudrate=baud, timeout=1) as ser:
        time.sleep(2)
        ser.write(command.encode("ascii"))
        ser.flush()
 
        print(f"Sent command: {command}")
 
        start_time = time.time()
        while time.time() - start_time < 5:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                state = parse_uart_state(line)
                if state is None:
                    print(line)
                else:
                    print_state(state)
 
 
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
 
    args = parser.parse_args()
    send_command(args.port, args.baud, args.command)
 
 
if __name__ == "__main__":
    main()
