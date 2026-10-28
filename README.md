# Ars2

ESP32-S3 (XIAO) sensor board. Reads two TCS34725 color sensors and two SparkFun OTOS
optical odometry sensors through a TCA9548A I2C mux, and reports them to the host over RS485.

Everything is fixed at compile time. Pins, mux ports, sensor names and OTOS offsets are in
[include/hw_config.h](include/hw_config.h).

* `src/sensors.cpp`: one FreeRTOS task (core 0) that brings up and reads all sensors every `SENSOR_PERIOD_MS`
* `src/comms.cpp`: RS485 protocol, polled from `loop()` (core 1). See `src/comms.h` for the commands
* `src/main.cpp`: setup and heartbeat

## Debug mode

Build and flash the `tasks_debug` env (`pio run -e tasks_debug -t upload`), or set `ARS2_DEBUG` to 1 in
`hw_config.h`. Every RS485 packet in and out is echoed to USB serial (`[RX]` / `[TX]`). Once the sensors
are up the board runs INIT itself, then prints a DATA packet every `DEBUG_DATA_PERIOD_MS`. These
self-run commands are tagged `[LOCAL RX]` / `[LOCAL TX]` and are never sent on the bus.
