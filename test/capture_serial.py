"""Capture USB diagnostics without deliberately resetting the sensor board.

Use --list-ports, then: capture_serial.py 300 /dev/cu.usbmodem... .
Each run creates a separate log; existing logs are never overwritten.
"""
import argparse
from datetime import datetime
from pathlib import Path
import time

import serial
from serial.tools import list_ports


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("seconds", type=float, nargs="?", default=300)
    parser.add_argument("port", nargs="?")
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.list_ports:
        for port in list_ports.comports():
            print(f"{port.device}: {port.description}")
        return
    if not args.port:
        parser.error("specify the board's USB port; use --list-ports to find it")
    if args.seconds <= 0:
        parser.error("seconds must be positive")

    started = datetime.now().astimezone()
    output = args.output or Path(__file__).with_name(
        f"serial_log_{started.strftime('%Y%m%d_%H%M%S_%f')}.txt"
    )
    if output.exists():
        parser.error(f"output already exists: {output}")
    # Set handshake lines before opening; do not deliberately toggle the ESP reset.
    connection = serial.Serial()
    connection.port = args.port
    connection.baudrate = 115200
    connection.timeout = 0.2
    connection.dtr = False
    connection.rts = False
    with output.open("x") as log:
        log.write(f"# Started {started.isoformat()} port={args.port} duration={args.seconds}s\n")
        with connection:
            print(f"Capturing {args.port} to {output.resolve()} for {args.seconds:.0f}s")
            print("Place the robot flat and still, then reset Ars2 to capture startup calibration.")
            beginning = time.monotonic()
            try:
                while time.monotonic() - beginning < args.seconds:
                    line = connection.readline().decode(errors="replace")
                    if line:
                        elapsed = time.monotonic() - beginning
                        log.write(f"[CAPTURE +{elapsed:.3f}s] {line}")
                        log.flush()
            except KeyboardInterrupt:
                print("Capture stopped.")
    print(f"Saved {output.resolve()}")


if __name__ == "__main__":
    main()
