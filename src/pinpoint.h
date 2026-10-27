#pragma once
#include <Arduino.h>

// Minimal I2C driver for the goBILDA Pinpoint odometry computer (address 0x31).
// Registers are 4 bytes, little-endian. Native units: mm and radians.

class Pinpoint {
public:
    enum Status : uint32_t {
        READY       = 1 << 0,
        CALIBRATING = 1 << 1,
        NO_X_POD    = 1 << 2,
        NO_Y_POD    = 1 << 3,
        IMU_RUNAWAY = 1 << 4,
        BAD_READ    = 1 << 5,
    };

    struct Pose {
        uint32_t status;
        float xMm, yMm, hRad;
    };

    // Configures pods and starts IMU calibration + position reset.
    // The robot must be still for ~0.25 s afterwards, until status reports READY.
    bool begin(float ticksPerMm, float xPodOffsetMm, float yPodOffsetMm,
               bool xReversed, bool yReversed);

    // One bulk read of status + position. False on I2C error.
    bool read(Pose& out);

private:
    static const uint8_t ADDR = 0x31;

    bool writeReg(uint8_t reg, const void* value);
    bool readRegs(uint8_t reg, void* out, size_t len);
};
