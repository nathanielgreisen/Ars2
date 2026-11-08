# Reset and stationary drift check

Build and upload **both** projects. The V5 now expects Ars2 to acknowledge the
reset protocol. Old Ars2 firmware will display `Reset unavailable: update Ars2
firmware` instead of quietly accepting an old origin.

## Upload

From the Ars2 directory:

```sh
~/.platformio/penv/bin/pio run -e tasks_debug -t upload
```

Build/upload the V5 program from `PushBackRobot/PushBackRobot` with your usual
PROS workflow. A build alone does not change either device.

## Capture one five-minute test

Close other serial monitors. Find the Ars2 USB port:

```sh
~/.platformio/penv/bin/python test/capture_serial.py --list-ports
```

Then start capture, substituting the actual port (Windows can use `COM3`, etc.):

```sh
~/.platformio/penv/bin/python test/capture_serial.py 300 /dev/cu.usbmodemYOUR_PORT
```

Each run saves a new timestamped log. The script sets DTR/RTS low before opening
and does not deliberately reboot the board. Once recording:

1. Put the robot flat on the test surface, with the sensors mounted as used in
   operation. Keep it completely still throughout calibration.
2. Press Ars2's reset button to capture its boot/calibration messages. USB may
   keep Ars2 powered when the robot battery is switched off; the reset button
   avoids confusing a V5-only restart with a sensor-board restart.
3. Start the V5 program. Its LCD should show `Keep still: resetting odometry`,
   then `Reset done: L=1 R=1 P=1`. A zero means that sensor failed initialization
   or the first read; it should display `bad` until it has a fresh valid reading.
4. Keep still for two minutes. Record the displayed x/y/heading at approximately
   0, 30, 60, and 120 seconds **after reset completes**. The `[POSE] reset_ms`
   field provides the timing in the serial log, with higher precision than LCD.
5. Stop and restart **only the V5 program**, leaving Ars2 powered. Confirm another
   `[RESET] host INIT received` and `[RESET] DONE`. Confirm all three sensor poses
   return near zero, then remain still for another minute.
6. Note any handling, motor vibration, temperature changes, or movement during
   the test. Do not repeatedly re-zero during a drift measurement.

Send the complete saved log, the LCD's reset result, and those readings. Also
include the test surface and each OTOS's height above it, whether both boards
were flat, and approximately when either device was reset.

## Reading the log

- `[OPTL] calibrateImu rc=0` and the corresponding `OPTR` line mean the library
  reported success. Configuration/scalar/reset steps each log their own result.
  A successful return does not establish that the robot was stationary.
- `[PNPT] reset/calibration READY ...` means the device finished calibration and
  reported a usable pose. Configuration failures and READY timeouts are logged.
- `[RESET] DONE ... OPTL=1 OPTR=1 PNPT=1` means reset finished and each named
  sensor supplied a fresh reading. This acknowledges completion, not just the
  receipt of the reset request.
- `[POSE] ... name=OPTL valid=1 ... x_in=... y_in=... h_deg=...` is the higher-precision
  Ars2 pose (three decimal places), before the V5 coordinate conversion. Compare
  **changes over time**; V5 x/y currently have the opposite sign at zero offsets.
- `reset_ms` is elapsed time since the last completed initialization/reset,
  `age_ms` is age of the last successful sensor read, and `ok`/`fail` are counts
  since that reset. Old coordinate numbers on a `valid=0` line are diagnostic
  history and are not sent as valid measurements.
- Optical `status ... bits=0x..` and Pinpoint `status=0x..` preserve device flags
  for investigating tracking, tilt, calibration, or encoder faults.
- Optical/Pinpoint readings older than 250ms are omitted by Ars2. The V5 also
  expires readings after 500ms, and never treats a serial timeout as a new sample.
- A failed initialization is retried on the next explicit program-start reset,
  not by silently recalibrating while the robot might be moving. A transient read
  failure after successful initialization recovers on the next good read.

## Follow-up connection test

While stationary, unplug **one sensor**. Its readings should become `bad`, with
the others continuing to update. Reconnect it and restart the V5 program while
still to re-establish a common zero. Send that log if detection or reset fails.

The validity flag checks communication, finite values, and freshness. It does
not perform statistical outlier rejection or establish position accuracy.

## Software checks (no hardware required)

From Ars2:

```sh
python3 test/host/run_tests.py
~/.platformio/penv/bin/pio run -e tasks_prod -e tasks_debug
```

From the outer PushBackRobot directory:

```sh
python3 tests/run_odometry_tests.py
```

The V5 native tests use the project's installed PROS headers. Normal robot builds
still run with `make` inside its inner `PushBackRobot` directory.
