#pragma once

// All hardware settings live here. Edit and rebuild.

// ---------------------------------------------------------------------------
// Mux ports (TCA9548A channel 0-7) for each sensor
// ---------------------------------------------------------------------------
#define CS1_PORT  8   // color sensor 1
#define CS2_PORT  9   // color sensor 2
#define OPTL_PORT 0   // OTOS left
#define OPTR_PORT 1   // OTOS right
#define PNPT_PORT 3   // goBILDA Pinpoint

// ---------------------------------------------------------------------------
// Sensor names (how each sensor is labeled in RS485 DATA replies)
// ---------------------------------------------------------------------------
#define CS1_NAME  "CS1"
#define CS2_NAME  "CS2"
#define OPTL_NAME "OPTL"
#define OPTR_NAME "OPTR"
#define PNPT_NAME "PNPT"

// Serial (USB debug and RS485 bus)
#define BAUDRATE 115200

// I2C + TCA9548A mux
#define I2C_SDA_PIN  5
#define I2C_SCL_PIN  6
#define I2C_FREQ_HZ  100000  // Pinpoint channel has no pull-ups; 400 kHz drops most reads
#define I2C_MUX_ADDR 0x70

// RS485 transceiver on Serial1
#define RS485_EN_PIN 4   // DE/RE: HIGH = transmit, LOW = receive
#define RS485_RX_PIN 3
#define RS485_TX_PIN 2

// Optical odometry (SparkFun OTOS) mounting offsets (in, in, deg)
#define OPTL_OFF_X -6.37f
#define OPTL_OFF_Y -0.73f
#define OPTL_OFF_H 0.0f

#define OPTR_OFF_X -6.37f
#define OPTR_OFF_Y 0.73f
#define OPTR_OFF_H 180.0f

#define OTOS_LINEAR_SCALAR  0.9976f
#define OTOS_ANGULAR_SCALAR 0.9938f

// goBILDA Pinpoint
// Direct-drive REV Through Bore encoders with 2-inch tracking wheels.
// Counts per revolution / tracking wheel circumference in mm.
#define PNPT_TICKS_PER_MM (8192.0f / (3.14159265f * 2.0f * 25.4f))
// With the current wiring, Pinpoint's X axis is robot right and its Y axis
// is robot forward. Each register measures its pod's offset on the other axis.
#define PNPT_X_POD_OFFSET_MM (-6.5f * 25.4f) // Lateral pod behind center (-Y).
#define PNPT_Y_POD_OFFSET_MM (6.5f * 25.4f)  // Longitudinal pod right of center (+X).
#define PNPT_X_REVERSED false
#define PNPT_Y_REVERSED true

// How often all sensors are read
#define SENSOR_PERIOD_MS 10
#define SENSOR_READING_TIMEOUT_MS 250
#define SENSOR_RESET_SETTLE_MS 250
#define PINPOINT_READY_TIMEOUT_MS 2000

// Debug mode: echoes RS485 packets and prints DATA without sending a reset.
// Pose diagnostics include precise readings, timestamps, and read failure counts.
// Enable here or build the tasks_debug env (pio run -e tasks_debug).
#ifndef ARS2_DEBUG
#define ARS2_DEBUG 1
#endif
#define DEBUG_DATA_PERIOD_MS 500
#define DEBUG_POSE_PERIOD_MS 1000

// Reserved for encoders (not used yet)
#define ENC_CLK_PIN 9
#define ENC_CS_PIN  43
#define ENC1_PIN    7
#define ENC2_PIN    8
#define ENC3_PIN    44
#define PHY_WH_DIST_CEN  3.056
#define PHY_HOR_DIST_CEN 0.832
#define PHY_TICK_P_IN    162.9746617
