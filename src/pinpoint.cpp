#include "pinpoint.h"
#include <Wire.h>

namespace {

enum Reg : uint8_t {
    DEVICE_ID      = 1,
    DEVICE_CONTROL = 4,
    TICKS_PER_MM   = 14,
    X_POD_OFFSET   = 15,
    Y_POD_OFFSET   = 16,
    BULK_READ      = 18,
};

enum Control : uint32_t {
    RESET_POS_AND_IMU = 1 << 1,
    Y_REVERSED        = 1 << 2,
    Y_FORWARD         = 1 << 3,
    X_REVERSED        = 1 << 4,
    X_FORWARD         = 1 << 5,
};

} // namespace

bool Pinpoint::writeReg(uint8_t reg, const void* value) {
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    Wire.write(static_cast<const uint8_t*>(value), 4);
    return Wire.endTransmission() == 0;
}

bool Pinpoint::readRegs(uint8_t reg, void* out, size_t len) {
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom(ADDR, len) != len) return false;
    Wire.readBytes(static_cast<uint8_t*>(out), len);
    return true;
}

bool Pinpoint::begin(float ticksPerMm, float xPodOffsetMm, float yPodOffsetMm,
                     bool xReversed, bool yReversed) {
    uint32_t id;
    if (!readRegs(DEVICE_ID, &id, 4)) return false;

    uint32_t xDir = xReversed ? X_REVERSED : X_FORWARD;
    uint32_t yDir = yReversed ? Y_REVERSED : Y_FORWARD;
    uint32_t reset = RESET_POS_AND_IMU;

    return writeReg(TICKS_PER_MM, &ticksPerMm)
        && writeReg(X_POD_OFFSET, &xPodOffsetMm)
        && writeReg(Y_POD_OFFSET, &yPodOffsetMm)
        && writeReg(DEVICE_CONTROL, &xDir)
        && writeReg(DEVICE_CONTROL, &yDir)
        && writeReg(DEVICE_CONTROL, &reset);
}

bool Pinpoint::read(Pose& out) {
    // Bulk layout: status, loop time, x enc, y enc, x, y, h, x vel, y vel, h vel
    uint8_t buf[40];
    if (!readRegs(BULK_READ, buf, sizeof(buf))) return false;

    memcpy(&out.status, buf + 0, 4);
    memcpy(&out.xMm, buf + 16, 4);
    memcpy(&out.yMm, buf + 20, 4);
    memcpy(&out.hRad, buf + 24, 4);
    return true;
}
