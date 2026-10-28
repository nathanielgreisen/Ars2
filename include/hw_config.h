#pragma once

// All hardware settings live here. Edit and rebuild.

// ---------------------------------------------------------------------------
// Mux ports (TCA9548A channel 0-7) for each sensor
// ---------------------------------------------------------------------------
#define CS1_PORT  8   // color sensor 1
#define CS2_PORT  9   // color sensor 2
#define OPTL_PORT 3   // OTOS left
#define OPTR_PORT 4   // OTOS right
#define PNPT_PORT 2   // goBILDA Pinpoint

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
#define OPTL_OFF_X -0.73f
#define OPTL_OFF_Y -6.37f
#define OPTL_OFF_H 0.0f

#define OPTR_OFF_X 0.73f
#define OPTR_OFF_Y -6.37f
#define OPTR_OFF_H 180.0f

#define OTOS_LINEAR_SCALAR  0.9976f
#define OTOS_ANGULAR_SCALAR 0.9938f

// goBILDA Pinpoint
// Ticks per mm: 4-bar pod = 19.89436789, swingarm pod = 13.26291192
#define PNPT_TICKS_PER_MM 19.89436789f
// X pod offset: how far left of center the forward-tracking pod is (mm, left +)
// Y pod offset: how far forward of center the strafe-tracking pod is (mm, forward +)
#define PNPT_X_POD_OFFSET_MM 0.0f
#define PNPT_Y_POD_OFFSET_MM 0.0f
#define PNPT_X_REVERSED false
#define PNPT_Y_REVERSED false

// How often all sensors are read
#define SENSOR_PERIOD_MS 10

// Debug mode: echoes every RS485 packet to USB serial, runs INIT once the sensors
// are up, and prints a DATA packet every DEBUG_DATA_PERIOD_MS without a host.
// Enable here or build the tasks_debug env (pio run -e tasks_debug).
#ifndef ARS2_DEBUG
#define ARS2_DEBUG 1
#endif
#define DEBUG_DATA_PERIOD_MS 500

// Reserved for encoders (not used yet)
#define ENC_CLK_PIN 9
#define ENC_CS_PIN  43
#define ENC1_PIN    7
#define ENC2_PIN    8
#define ENC3_PIN    44
#define PHY_WH_DIST_CEN  3.056
#define PHY_HOR_DIST_CEN 0.832
#define PHY_TICK_P_IN    162.9746617
