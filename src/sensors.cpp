#include "sensors.h"
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include <hw_config.h>
#include "pinpoint.h"

// Only the sensor task touches I2C, so the bus needs no locking.
// Readings are shared with the comms code through a spinlock.

namespace {

enum { CS1, CS2, OPTL, OPTR, PNPT };

const uint8_t PORTS[SENSOR_COUNT] = {CS1_PORT, CS2_PORT, OPTL_PORT, OPTR_PORT, PNPT_PORT};

Adafruit_TCS34725 cs1(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_60X);
Adafruit_TCS34725 cs2(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_60X);
QwiicOTOS optL;
QwiicOTOS optR;
Pinpoint pinpoint;

bool online[SENSOR_COUNT] = {};
volatile bool ready = false;

SensorReading latest[SENSOR_COUNT] = {
    {CS1_NAME, 0, 0, 0, false},
    {CS2_NAME, 0, 0, 0, false},
    {OPTL_NAME, 0, 0, 0, false},
    {OPTR_NAME, 0, 0, 0, false},
    {PNPT_NAME, 0, 0, 0, false},
};
portMUX_TYPE latestLock = portMUX_INITIALIZER_UNLOCKED;

bool selectPort(uint8_t port) {
    Wire.beginTransmission(I2C_MUX_ADDR);
    Wire.write(1 << port);
    return Wire.endTransmission() == 0;
}

void publish(int i, int32_t a, int32_t b, int32_t c) {
    portENTER_CRITICAL(&latestLock);
    latest[i].a = a;
    latest[i].b = b;
    latest[i].c = c;
    latest[i].valid = true;
    portEXIT_CRITICAL(&latestLock);
}

bool beginColor(Adafruit_TCS34725& cs, int i) {
    return selectPort(PORTS[i]) && cs.begin();
}

bool beginOptical(QwiicOTOS& otos, int i, float x, float y, float h) {
    if (!selectPort(PORTS[i]) || !otos.begin()) return false;

    sfe_otos_pose2d_t offset = {x, y, h};
    otos.setOffset(offset);
    otos.calibrateImu();
    otos.setLinearScalar(OTOS_LINEAR_SCALAR);
    otos.setAngularScalar(OTOS_ANGULAR_SCALAR);
    otos.resetTracking();
    return true;
}

void readColor(Adafruit_TCS34725& cs, int i) {
    if (!online[i] || !selectPort(PORTS[i])) return;

    uint16_t r, g, b, c;
    cs.getRawData(&r, &g, &b, &c);
    publish(i, r, g, b);
}

void readOptical(QwiicOTOS& otos, int i) {
    if (!online[i] || !selectPort(PORTS[i])) return;

    sfe_otos_pose2d_t pos;
    if (otos.getPosition(pos) != ksfTkErrOk) return;
    publish(i, (int32_t)(pos.x * 100.0f), (int32_t)(pos.y * 100.0f), (int32_t)pos.h);
}

bool beginPinpoint() {
    return selectPort(PORTS[PNPT])
        && pinpoint.begin(PNPT_TICKS_PER_MM, PNPT_X_POD_OFFSET_MM, PNPT_Y_POD_OFFSET_MM,
                          PNPT_X_REVERSED, PNPT_Y_REVERSED);
}

void report(int i, const char* name) {
    Serial.printf("[%s] port %u: %s\n", name, PORTS[i], online[i] ? "OK" : "NOT FOUND");
}

// Debug mode: Pinpoint read success counts, printed once a second
uint32_t pnptOk = 0, pnptFail = 0;

void pinpointStats() {
#if ARS2_DEBUG
    static uint32_t last = 0;
    uint32_t now = millis();
    if (now - last < 1000) return;
    last = now;
    Serial.printf("[%s] reads ok=%lu fail=%lu\n", PNPT_NAME, (unsigned long)pnptOk, (unsigned long)pnptFail);
    pnptOk = pnptFail = 0;
#endif
}

// Debug mode: says why the Pinpoint isn't publishing, at most once a second
void pinpointDebug(const char* why, uint32_t status = 0) {
#if ARS2_DEBUG
    pnptFail++;
    static uint32_t last = 0;
    uint32_t now = millis();
    if (now - last < 1000) return;
    last = now;
    Serial.printf("[%s] %s (status=0x%02lx)\n", PNPT_NAME, why, (unsigned long)status);
#endif
}

void readPinpoint() {
    pinpointStats();

    // Retry bring-up once a second in case the Pinpoint booted after us
    static uint32_t lastRetry = 0;
    if (!online[PNPT]) {
        if (millis() - lastRetry < 1000) return;
        lastRetry = millis();
        online[PNPT] = beginPinpoint();
        if (!online[PNPT]) { pinpointDebug("not found on I2C"); return; }
        report(PNPT, PNPT_NAME);
    }
    if (!selectPort(PORTS[PNPT])) { pinpointDebug("mux select failed"); return; }

    Pinpoint::Pose pose;
    if (!pinpoint.read(pose)) { pinpointDebug("bulk read failed"); return; }
    if (!(pose.status & Pinpoint::READY)) {
        // 0x02 calibrating, 0x04/0x08/0x0C X/Y/both pods not detected, 0x10 IMU runaway
        pinpointDebug("not READY", pose.status);
        return;
    }
    if (isnan(pose.xMm) || isnan(pose.yMm) || isnan(pose.hRad)) {
        pinpointDebug("NaN pose", pose.status);
        return;
    }

    pnptOk++;

    // Same units as the OTOS: inches * 100 and degrees
    publish(PNPT, (int32_t)(pose.xMm / 25.4f * 100.0f), (int32_t)(pose.yMm / 25.4f * 100.0f),
            (int32_t)(pose.hRad * RAD_TO_DEG));
}

// Debug mode: lists every I2C address that answers, per mux port
void scanBus() {
#if ARS2_DEBUG
    for (int port = -1; port < 8; port++) {
        if (port < 0) {
            Wire.beginTransmission(I2C_MUX_ADDR);
            Wire.write(0); // all mux ports off: shows devices on the main bus
            Wire.endTransmission();
        } else if (!selectPort(port)) {
            Serial.println("[SCAN] mux not responding");
            return;
        }

        Serial.printf("[SCAN] %s:", port < 0 ? "main bus" : String("port " + String(port)).c_str());
        for (uint8_t addr = 1; addr < 127; addr++) {
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

    online[CS1]  = beginColor(cs1, CS1);
    online[CS2]  = beginColor(cs2, CS2);
    online[OPTL] = beginOptical(optL, OPTL, OPTL_OFF_X, OPTL_OFF_Y, OPTL_OFF_H);
    online[OPTR] = beginOptical(optR, OPTR, OPTR_OFF_X, OPTR_OFF_Y, OPTR_OFF_H);
    online[PNPT] = beginPinpoint();

    report(CS1, CS1_NAME);
    report(CS2, CS2_NAME);
    report(OPTL, OPTL_NAME);
    report(OPTR, OPTR_NAME);
    report(PNPT, PNPT_NAME);
    ready = true;

    TickType_t lastWake = xTaskGetTickCount();
    for (;;) {
        readColor(cs1, CS1);
        readColor(cs2, CS2);
        readOptical(optL, OPTL);
        readOptical(optR, OPTR);
        readPinpoint();
#if ARS2_DEBUG
        // Repeat the scan so it's visible even if the monitor connected late
        static uint32_t lastScan = 0;
        if (millis() - lastScan >= 10000) {
            lastScan = millis();
            scanBus();
        }
#endif
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

} // namespace

void sensorsBegin() {
    // Core 0 keeps sensor reads off the core that answers RS485 requests
    xTaskCreatePinnedToCore(sensorTask, "sensors", 8192, nullptr, 1, nullptr, 0);
}

bool sensorsReady() {
    return ready;
}

void sensorsSnapshot(SensorReading out[SENSOR_COUNT]) {
    portENTER_CRITICAL(&latestLock);
    memcpy(out, latest, sizeof(latest));
    portEXIT_CRITICAL(&latestLock);
}
