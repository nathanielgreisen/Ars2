"""Saves the board's USB serial output to test/serial_log.txt.

Usage: ~/.platformio/penv/bin/python test/capture_serial.py [seconds] [port]
Defaults: 30 seconds, /dev/ttyACM0. Close the PlatformIO serial monitor first.
"""
import sys
import time
from pathlib import Path

import serial

seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 30
port = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyACM0"
out = Path(__file__).with_name("serial_log.txt")

end = time.time() + seconds
with serial.Serial(port, 115200, timeout=0.2) as ser, out.open("w") as f:
    print(f"Logging {port} to {out} for {seconds:.0f} s")
    while time.time() < end:
        line = ser.readline().decode(errors="replace")
        if line:
            f.write(line)
            f.flush()
print("Done")
