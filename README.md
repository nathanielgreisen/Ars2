# Ars2

ESP32-S3 (XIAO) sensor board. Reads two TCS34725 color sensors and two SparkFun OTOS
optical odometry sensors through a TCA9548A I2C mux, and reports them to the host over RS485.

Everything is fixed at compile time. Pins, mux ports, sensor names and OTOS offsets are in
[include/hw_config.h](include/hw_config.h).

* `src/sensors.cpp`: one FreeRTOS task (core 0) that brings up and reads all sensors every `SENSOR_PERIOD_MS`
* `src/comms.cpp`: RS485 protocol, polled from `loop()` (core 1). See `src/comms.h` for the commands
* `src/main.cpp`: setup and heartbeat
