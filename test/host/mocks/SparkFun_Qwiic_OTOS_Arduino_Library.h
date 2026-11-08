#pragma once
#include "Arduino.h"
using sfTkError_t = int;
constexpr int ksfTkErrOk = 0;
constexpr int kSfeOtosLinearUnitInches = 0, kSfeOtosAngularUnitDegrees = 0;
struct sfe_otos_pose2d_t { float x, y, h; };
struct sfe_otos_status_t { uint8_t value; };
struct MockOptical {
    bool present = true, calibrationFails = false, resetFails = false, readFails = false;
    unsigned calibrations = 0, resets = 0;
    sfe_otos_pose2d_t pose{9, 8, 7}, offset{};
};
inline MockOptical mockOpticals[2];
inline unsigned nextOptical = 0;
inline std::function<void()> duringCalibration;
class QwiicOTOS {
    unsigned index = nextOptical++;
public:
    bool begin() { return mockOpticals[index].present; }
    void setLinearUnit(int) {}
    void setAngularUnit(int) {}
    int setOffset(sfe_otos_pose2d_t& value) { mockOpticals[index].offset = value; return 0; }
    int calibrateImu() {
        ++mockOpticals[index].calibrations;
        if (duringCalibration) duringCalibration();
        delay(620);
        return mockOpticals[index].calibrationFails ? -1 : 0;
    }
    int setLinearScalar(float) { return 0; }
    int setAngularScalar(float) { return 0; }
    int resetTracking() {
        ++mockOpticals[index].resets;
        if (mockOpticals[index].resetFails) return -1;
        mockOpticals[index].pose = {0, 0, 0};
        return 0;
    }
    int getPosition(sfe_otos_pose2d_t& value) {
        if (mockOpticals[index].readFails) return -1;
        value = mockOpticals[index].pose;
        return 0;
    }
    int getStatus(sfe_otos_status_t& value) { value.value = 0; return 0; }
};
