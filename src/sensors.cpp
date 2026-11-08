#include "sensors.h"
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include <cmath>
#include <limits>
#include <hw_config.h>
#include "pinpoint.h"

// Only this task touches I2C. Commands queue work through latestLock.
namespace {

enum { CS1, CS2, OPTL, OPTR, PNPT };
const uint8_t PORTS[SENSOR_COUNT] = {CS1_PORT, CS2_PORT, OPTL_PORT, OPTR_PORT, PNPT_PORT};

Adafruit_TCS34725 cs1(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_60X);
Adafruit_TCS34725 cs2(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_60X);
QwiicOTOS optL;
QwiicOTOS optR;
Pinpoint pinpoint;
bool online[SENSOR_COUNT] = {};
bool ready = false;
SensorResetStatus resetStatus;

SensorReading latest[SENSOR_COUNT] = {
    {CS1_NAME, 0, 0, 0, false}, {CS2_NAME, 0, 0, 0, false},
    {OPTL_NAME, 0, 0, 0, false}, {OPTR_NAME, 0, 0, 0, false},
    {PNPT_NAME, 0, 0, 0, false},
};
uint32_t updatedAt[SENSOR_COUNT] = {};
portMUX_TYPE latestLock = portMUX_INITIALIZER_UNLOCKED;

struct Diagnostics {
    uint32_t ok = 0, fail = 0, lastErrorMs = 0;
    float x = 0, y = 0, heading = 0;
};
Diagnostics diagnostics[SENSOR_COUNT]; // Accessed only by the sensor task.
uint32_t resetAt = 0;
uint32_t pinpointStatus = 0;

bool selectPort(uint8_t port) {
    if (port > 7) return false;
    Wire.beginTransmission(I2C_MUX_ADDR);
    Wire.write(uint8_t(1u << port));
    return Wire.endTransmission() == 0;
}

void unavailable(int i, const char* why, int error = 0) {
    portENTER_CRITICAL(&latestLock);
    latest[i].valid = false;
    portEXIT_CRITICAL(&latestLock);
    auto& d = diagnostics[i];
    ++d.fail;
    const uint32_t now = millis();
    if (d.fail == 1 || now - d.lastErrorMs >= 1000) {
        d.lastErrorMs = now;
        Serial.printf("[%s] unavailable: %s (code=%d)\r\n", latest[i].name, why, error);
    }
}

void publish(int i, int32_t a, int32_t b, int32_t c) {
    const uint32_t now = millis();
    portENTER_CRITICAL(&latestLock);
    latest[i] = {latest[i].name, a, b, c, true};
    updatedAt[i] = now;
    portEXIT_CRITICAL(&latestLock);
}

void publishPose(int i, float xInches, float yInches, float headingDegrees) {
    const double scaledX = double(xInches * 100.0f);
    const double scaledY = double(yInches * 100.0f);
    const double low = std::numeric_limits<int32_t>::min();
    const double high = std::numeric_limits<int32_t>::max();
    if (!std::isfinite(scaledX) || !std::isfinite(scaledY) || !std::isfinite(headingDegrees)
        || scaledX < low || scaledX > high || scaledY < low || scaledY > high
        || double(headingDegrees) < low || double(headingDegrees) > high) {
        unavailable(i, "pose outside wire-format range");
        return;
    }
    auto& d = diagnostics[i];
    ++d.ok;
    d.x = xInches;
    d.y = yInches;
    d.heading = headingDegrees;
    publish(i, int32_t(scaledX), int32_t(scaledY), int32_t(headingDegrees));
}

bool beginColor(Adafruit_TCS34725& cs, int i) {
    return selectPort(PORTS[i]) && cs.begin();
}

bool opticalStep(int i, const char* step, sfTkError_t result) {
    Serial.printf("[%s] %s rc=%d\r\n", latest[i].name, step, int(result));
    return result == ksfTkErrOk;
}

bool beginOptical(QwiicOTOS& otos, int i, float x, float y, float h) {
    if (!selectPort(PORTS[i]) || !otos.begin()) {
        unavailable(i, "begin/mux failed");
        return false;
    }
    // Set units explicitly before interpreting mounting offsets.
    otos.setLinearUnit(kSfeOtosLinearUnitInches);
    otos.setAngularUnit(kSfeOtosAngularUnitDegrees);
    sfe_otos_pose2d_t offset = {x, y, h};
    Serial.printf("[%s] offset=(%.3f,%.3f,%.3f) in,in,deg; keep robot still\r\n",
                  latest[i].name, x, y, h);
    if (!opticalStep(i, "setOffset", otos.setOffset(offset))
        || !opticalStep(i, "calibrateImu", otos.calibrateImu())
        || !opticalStep(i, "setLinearScalar", otos.setLinearScalar(OTOS_LINEAR_SCALAR))
        || !opticalStep(i, "setAngularScalar", otos.setAngularScalar(OTOS_ANGULAR_SCALAR))
        || !opticalStep(i, "resetTracking", otos.resetTracking())) {
        unavailable(i, "initialization failed; restart program to retry");
        return false;
    }
    delay(20); // Let the device apply the reset before its first reading.
    return true;
}

void readColor(Adafruit_TCS34725& cs, int i) {
    if (!online[i] || !selectPort(PORTS[i])) return;
    uint16_t r, g, b, c;
    cs.getRawData(&r, &g, &b, &c);
    publish(i, r, g, b);
}

void readOptical(QwiicOTOS& otos, int i) {
    if (!online[i]) return;
    if (!selectPort(PORTS[i])) {
        unavailable(i, "mux select failed");
        return;
    }
    sfe_otos_pose2d_t pos;
    const auto result = otos.getPosition(pos);
    if (result != ksfTkErrOk) {
        unavailable(i, "getPosition failed", int(result));
        return;
    }
    if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.h)) {
        unavailable(i, "non-finite pose");
        return;
    }
    publishPose(i, pos.x, pos.y, pos.h);
}

bool usablePinpointPose(const Pinpoint::Pose& pose) {
    constexpr uint32_t faults = Pinpoint::CALIBRATING | Pinpoint::NO_X_POD
        | Pinpoint::NO_Y_POD | Pinpoint::IMU_RUNAWAY | Pinpoint::BAD_READ;
    return (pose.status & Pinpoint::READY) && !(pose.status & faults)
        && std::isfinite(pose.xMm) && std::isfinite(pose.yMm) && std::isfinite(pose.hRad);
}

bool beginPinpoint() {
    if (!selectPort(PORTS[PNPT])
        || !pinpoint.begin(PNPT_TICKS_PER_MM, PNPT_X_POD_OFFSET_MM, PNPT_Y_POD_OFFSET_MM,
                           PNPT_X_REVERSED, PNPT_Y_REVERSED)) {
        unavailable(PNPT, "configuration/reset write failed");
        return false;
    }
    Serial.printf("[PNPT] config ticks_per_mm=%.3f x_offset_mm=%.1f y_offset_mm=%.1f\r\n",
                  PNPT_TICKS_PER_MM, PNPT_X_POD_OFFSET_MM, PNPT_Y_POD_OFFSET_MM);
    // A successful write starts calibration; it does not mean calibration is done.
    const uint32_t started = millis();
    delay(20);
    while (millis() - started < PINPOINT_READY_TIMEOUT_MS) {
        Pinpoint::Pose pose;
        if (pinpoint.read(pose)) {
            pinpointStatus = pose.status;
            if (usablePinpointPose(pose)) {
                Serial.printf("[PNPT] reset/calibration READY after %lu ms (status=0x%02lx)\r\n",
                              (unsigned long)(millis() - started), (unsigned long)pose.status);
                return true;
            }
        }
        delay(10);
    }
    unavailable(PNPT, "READY timeout after reset", int(pinpointStatus));
    return false;
}

void readPinpoint() {
    if (!online[PNPT]) return;
    if (!selectPort(PORTS[PNPT])) {
        unavailable(PNPT, "mux select failed");
        return;
    }
    Pinpoint::Pose pose;
    if (!pinpoint.read(pose)) {
        unavailable(PNPT, "bulk read failed");
        return;
    }
    pinpointStatus = pose.status;
    if (!usablePinpointPose(pose)) {
        unavailable(PNPT, "not READY or non-finite pose", int(pose.status));
        return;
    }
    // Pinpoint integrates right/forward with positive CCW yaw. The V5 wire
    // convention for PNPT is right/back at reset, so negate Y after integration.
    publishPose(PNPT, pose.xMm / 25.4f, -pose.yMm / 25.4f, pose.hRad * RAD_TO_DEG);
}

void initializePoseSensors() {
    // Recalibration is explicit: never automatically calibrate a moving robot
    // following a transient read failure.
    for (int i = OPTL; i <= PNPT; ++i) diagnostics[i] = {};
    Serial.println("[RESET] calibrating: keep robot flat and stationary");
    delay(SENSOR_RESET_SETTLE_MS);
    online[OPTL] = beginOptical(optL, OPTL, OPTL_OFF_X, OPTL_OFF_Y, OPTL_OFF_H);
    online[OPTR] = beginOptical(optR, OPTR, OPTR_OFF_X, OPTR_OFF_Y, OPTR_OFF_H);
    online[PNPT] = beginPinpoint();
    // Publish only readings acquired after all reset/calibration work.
    readOptical(optL, OPTL);
    readOptical(optR, OPTR);
    readPinpoint();
    resetAt = millis();
}

void serviceReset() {
    portENTER_CRITICAL(&latestLock);
    const bool requested = resetStatus.state == ResetState::Pending;
    if (requested) resetStatus.state = ResetState::Running;
    portEXIT_CRITICAL(&latestLock);
    if (!requested) return;

    initializePoseSensors();
    portENTER_CRITICAL(&latestLock);
    resetStatus.optL = latest[OPTL].valid;
    resetStatus.optR = latest[OPTR].valid;
    resetStatus.pinpoint = latest[PNPT].valid;
    resetStatus.state = ResetState::Complete;
    const auto result = resetStatus;
    portEXIT_CRITICAL(&latestLock);
    Serial.printf("[RESET] DONE t_ms=%lu OPTL=%u OPTR=%u PNPT=%u\r\n",
                  (unsigned long)resetAt, result.optL, result.optR, result.pinpoint);
}

void printDiagnostics() {
#if ARS2_DEBUG
    static uint32_t last = 0;
    const uint32_t now = millis();
    if (now - last < DEBUG_POSE_PERIOD_MS) return;
    last = now;
    SensorReading snapshot[SENSOR_COUNT];
    sensorsSnapshot(snapshot);
    for (int i = OPTL; i <= PNPT; ++i) {
        const auto& d = diagnostics[i];
        Serial.printf("[POSE] t_ms=%lu reset_ms=%lu name=%s valid=%u age_ms=%lu "
                      "x_in=%.3f y_in=%.3f h_deg=%.3f ok=%lu fail=%lu\r\n",
                      (unsigned long)now, (unsigned long)(now - resetAt), latest[i].name,
                      snapshot[i].valid, (unsigned long)(now - updatedAt[i]),
                      d.x, d.y, d.heading, (unsigned long)d.ok, (unsigned long)d.fail);
    }
    for (int i : {OPTL, OPTR}) {
        sfe_otos_status_t status;
        auto& otos = i == OPTL ? optL : optR;
        if (online[i] && selectPort(PORTS[i])) {
            const auto rc = otos.getStatus(status);
            Serial.printf("[%s] status rc=%d bits=0x%02x\r\n", latest[i].name,
                          int(rc), rc == ksfTkErrOk ? status.value : 0);
        }
    }
    Serial.printf("[PNPT] status=0x%02lx\r\n", (unsigned long)pinpointStatus);
#endif
}

void scanBus() {
#if ARS2_DEBUG
    // Scan once at boot. Repeated scans would interrupt the 10ms read loop.
    for (int port = -1; port < 8; ++port) {
        if (port < 0) {
            Wire.beginTransmission(I2C_MUX_ADDR);
            Wire.write(0);
            Wire.endTransmission();
        } else if (!selectPort(port)) {
            Serial.println("[SCAN] mux not responding");
            return;
        }
        Serial.printf("[SCAN] %s:", port < 0 ? "main bus" : String("port " + String(port)).c_str());
        for (uint8_t addr = 1; addr < 127; ++addr) {
            if (addr == I2C_MUX_ADDR) continue;
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0) Serial.printf(" 0x%02X", addr);
        }
        Serial.println();
    }
#endif
}

void sensorTask(void*) {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
    scanBus();
    online[CS1] = beginColor(cs1, CS1);
    online[CS2] = beginColor(cs2, CS2);
    initializePoseSensors();
    portENTER_CRITICAL(&latestLock);
    ready = true;
    portEXIT_CRITICAL(&latestLock);

    TickType_t lastWake = xTaskGetTickCount();
    for (;;) {
        serviceReset();
        readColor(cs1, CS1);
        readColor(cs2, CS2);
        readOptical(optL, OPTL);
        readOptical(optR, OPTR);
        readPinpoint();
        printDiagnostics();
        // Rebase after a long reset so there is no burst of catch-up iterations.
        if (xTaskGetTickCount() - lastWake > pdMS_TO_TICKS(SENSOR_PERIOD_MS)) {
            lastWake = xTaskGetTickCount();
        }
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

} // namespace

void sensorsBegin() {
    xTaskCreatePinnedToCore(sensorTask, "sensors", 8192, nullptr, 1, nullptr, 0);
}

bool sensorsReady() {
    portENTER_CRITICAL(&latestLock);
    const bool result = ready;
    portEXIT_CRITICAL(&latestLock);
    return result;
}

void sensorsRequestReset() {
    portENTER_CRITICAL(&latestLock);
    if (resetStatus.state != ResetState::Pending && resetStatus.state != ResetState::Running) {
        resetStatus = {};
        resetStatus.state = ResetState::Pending;
        for (int i = OPTL; i <= PNPT; ++i) {
            latest[i].a = latest[i].b = latest[i].c = 0;
            latest[i].valid = false;
        }
    }
    portEXIT_CRITICAL(&latestLock);
    Serial.println("[RESET] host INIT received");
}

SensorResetStatus sensorsResetStatus() {
    portENTER_CRITICAL(&latestLock);
    const auto result = resetStatus;
    portEXIT_CRITICAL(&latestLock);
    return result;
}

void sensorsSnapshot(SensorReading out[SENSOR_COUNT]) {
    portENTER_CRITICAL(&latestLock);
    const uint32_t now = millis();
    memcpy(out, latest, sizeof(latest));
    const bool resetting = resetStatus.state == ResetState::Pending || resetStatus.state == ResetState::Running;
    for (size_t i = 0; i < SENSOR_COUNT; ++i) {
        if (now - updatedAt[i] > SENSOR_READING_TIMEOUT_MS || (resetting && i >= OPTL)) {
            out[i].valid = false;
        }
    }
    portEXIT_CRITICAL(&latestLock);
}
