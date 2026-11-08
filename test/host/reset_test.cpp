// Run the real sensor and command implementation against simulated devices.
// Including the sensor source lets this deterministic harness service one queued
// job without launching the firmware's infinite FreeRTOS loop.
#include "../../src/sensors.cpp"
#include "../../src/comms.cpp"
#include <cassert>
#include <iostream>
#include <limits>

int main() {
    SensorReading readings[SENSOR_COUNT];
    sensorsSnapshot(readings);
    assert(!readings[OPTL].valid && !readings[PNPT].valid);
    assert(!selectPort(8) && !selectPort(9));

    const int transactions = mockTransactions;
    commsRunLocal("INIT");
    assert(mockTransactions == transactions); // Command thread never touches I2C.
    assert(sensorsResetStatus().state == ResetState::Pending);
    commsRunLocal("RSET");
    assert(Serial.text.find("<ACK><RSET>(PENDING)") != std::string::npos);
    sensorsRequestReset(); // Duplicate pending requests are coalesced.
    duringCalibration = [] {
        SensorReading snapshot[SENSOR_COUNT];
        sensorsSnapshot(snapshot);
        assert(!snapshot[OPTL].valid && !snapshot[OPTR].valid && !snapshot[PNPT].valid);
        sensorsRequestReset(); // Duplicate running requests do not create another job.
    };
    serviceReset();
    duringCalibration = {};
    assert(mockOpticals[0].calibrations == 1 && mockOpticals[1].calibrations == 1);
    assert(mockPinResets == 1);
    assert(sensorsResetStatus().state == ResetState::Complete);
    sensorsSnapshot(readings);
    for (int i : {OPTL, OPTR, PNPT}) {
        assert(readings[i].valid && readings[i].a == 0 && readings[i].b == 0 && readings[i].c == 0);
    }
    commsRunLocal("RSET");
    commsRunLocal("RSET");
    assert(mockPinResets == 1); // Polling completion never resets again.
    assert(Serial.text.find("(DONE,OPTL=1,OPTR=1,PNPT=1)") != std::string::npos);
    assert(mockOpticals[0].offset.y == OPTL_OFF_Y && mockOpticals[1].offset.h == OPTR_OFF_H);
    assert(mockPinConfig[0] == PNPT_TICKS_PER_MM);
    assert(std::abs(mockPinConfig[1] + 165.1f) < 0.001f);
    assert(std::abs(mockPinConfig[2] - 165.1f) < 0.001f);
    assert(mockPinDirections == ((1u << 5) | (1u << 2))); // X forward, Y reversed.

    // A Pinpoint pose right and forward of reset is sent as right and back.
    mockPinPose[0] = 4.0f * 25.4f;
    mockPinPose[1] = 8.0f * 25.4f;
    mockPinPose[2] = 30.5f / RAD_TO_DEG;
    readPinpoint();
    sensorsSnapshot(readings);
    assert(readings[PNPT].valid && readings[PNPT].a == 400
           && readings[PNPT].b == -800 && readings[PNPT].c == 30);

    mockOpticals[0].readFails = true;
    readOptical(optL, OPTL);
    sensorsSnapshot(readings);
    assert(!readings[OPTL].valid && readings[OPTR].valid);
    mockOpticals[0].readFails = false;
    readOptical(optL, OPTL);
    sensorsSnapshot(readings);
    assert(readings[OPTL].valid);
    mockOpticals[0].pose.x = std::numeric_limits<float>::quiet_NaN();
    readOptical(optL, OPTL);
    sensorsSnapshot(readings);
    assert(!readings[OPTL].valid);

    publishPose(OPTL, std::numeric_limits<float>::max(), 0, 0);
    sensorsSnapshot(readings);
    assert(!readings[OPTL].valid);

    // Restarting the host resets accumulated poses, even with the MCU still up.
    mockOpticals[0].pose = {100, 200, 90};
    mockOpticals[1].pose = {300, 400, 120};
    sensorsRequestReset();
    serviceReset();
    sensorsSnapshot(readings);
    assert(readings[OPTL].valid && readings[OPTL].a == 0);
    assert(readings[OPTR].valid && readings[OPTR].a == 0);
    mockTime += SENSOR_READING_TIMEOUT_MS + 1;
    sensorsSnapshot(readings);
    assert(!readings[OPTL].valid && !readings[OPTR].valid && !readings[PNPT].valid);

    mockOpticals[1].calibrationFails = true;
    sensorsRequestReset();
    serviceReset();
    assert(sensorsResetStatus().optL && !sensorsResetStatus().optR && sensorsResetStatus().pinpoint);
    Serial.text.clear();
    commsRunLocal("DATA");
    assert(Serial.text.find("OPTR(") == std::string::npos);
    assert(Serial.text.find("OPTL(") != std::string::npos);
    mockOpticals[1].calibrationFails = false;

    mockOpticals[1].resetFails = true;
    sensorsRequestReset();
    serviceReset();
    assert(!sensorsResetStatus().optR);
    mockOpticals[1].resetFails = false;
    mockPinNeverReady = true;
    sensorsRequestReset();
    const auto before = millis();
    serviceReset();
    assert(!sensorsResetStatus().pinpoint && millis() - before < 5000);
    mockPinNeverReady = false;
    sensorsRequestReset();
    serviceReset();
    assert(sensorsResetStatus().pinpoint && sensorsResetStatus().optR);
    mockShortRead = true;
    readPinpoint();
    sensorsSnapshot(readings);
    assert(!readings[PNPT].valid);
    mockShortRead = false;
    readPinpoint();
    sensorsSnapshot(readings);
    assert(readings[PNPT].valid);

    mockTime = UINT32_MAX - 20;
    publishPose(OPTL, 0, 0, 0);
    mockTime += 30;
    sensorsSnapshot(readings);
    assert(readings[OPTL].valid);
    mockTime += SENSOR_READING_TIMEOUT_MS + 1;
    sensorsSnapshot(readings);
    assert(!readings[OPTL].valid);
    std::cout << "PASS: queued reset, I2C ownership, duplicate requests, polling, "
                 "zero origin, Pinpoint configuration and pose, calibration/reset failures, "
                 "READY timeout, stale readings, recovery\n";
}
