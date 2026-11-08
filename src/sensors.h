#pragma once
#include <Arduino.h>

constexpr size_t SENSOR_COUNT = 5;

enum class ResetState { Idle, Pending, Running, Complete };

struct SensorResetStatus {
    ResetState state = ResetState::Idle;
    bool optL = false;
    bool optR = false;
    bool pinpoint = false;
};

struct SensorReading {
    const char* name;
    int32_t a, b, c;   // color: R, G, B   optical/pinpoint: x*100 (in), y*100 (in), heading (deg)
    bool valid;        // false until the sensor has produced a good read
};

// Starts the background task that brings up and continuously reads all sensors.
// Returns immediately; sensors come online over ~1.5 s (OTOS IMU calibration).
void sensorsBegin();

// True once every sensor has been brought up (found or not).
bool sensorsReady();

// Copies the latest readings into out.
void sensorsSnapshot(SensorReading out[SENSOR_COUNT]);

// Queues calibration + tracking reset on the I2C-owning sensor task.
// Repeated requests while pending/running are coalesced. Keep the robot still.
void sensorsRequestReset();
SensorResetStatus sensorsResetStatus();
