#pragma once
#include "../lib/sensor_base.h"
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>

class OpticalSensor : public SensorBase {
public:
    OpticalSensor(const char* name, uint8_t channel, float offsetX, float offsetY, float offsetH)
        : SensorBase(name, channel), off_x(offsetX), off_y(offsetY), off_h(offsetH), initialized(false)
    {}

    void setup() override {
        I2CUtils::i2cLock();
        bool muxOK = I2CUtils::selectChannel(_muxChannel);

        if (!muxOK) {
            Serial.printf("[%s] MUX select failed\n", _name);
            I2CUtils::i2cUnlock();
            initialized = false;
            return;
        }

        if (!otos.begin()) {
            Serial.printf("[%s] OTOS not found!\n", _name);
            I2CUtils::i2cUnlock();
            initialized = false;
            return;
        }

        Serial.printf("OTOS on CH%u initialized OK\n", _muxChannel);

        offset.x = off_x;
        offset.y = off_y;
        offset.h = off_h;

        otos.setOffset(offset);
        otos.calibrateImu();
        otos.setLinearScalar(0.9976f);
        otos.setAngularScalar(0.9938f);
        otos.resetTracking();

        initialized = true;
        I2CUtils::i2cUnlock();
    }

    void readRaw() override {
        //if (!initialized) return;

        //I2CUtils::i2cLock();

        //if (!I2CUtils::selectChannel(_muxChannel)) {
            //I2CUtils::i2cUnlock();
            //return;
        //}

        otos.getPosition(pos);
        //I2CUtils::i2cUnlock();

        TelemetryPacket p{};
        p.name = _name;
        p.a = (int32_t)(pos.x * 100.0f);
        p.b = (int32_t)(pos.y * 100.0f);
        p.c = (int32_t)(pos.h);
        p.ms = millis();
        TelemetryBus::publish(p);
    }

    // Wacky Print statement so we dont get the encoding error
    void debugPrint() override {
        Serial.printf("[%s] ", _name);
        Serial.print("X=");
        Serial.print(pos.x);
        Serial.print("Y=");
        Serial.print(pos.y);
        Serial.print("H=");
        Serial.print(pos.h);
        Serial.print("\n");
    }

    sfe_otos_pose2d_t pos;

private:
    float off_x;
    float off_y;
    float off_h;

    bool initialized;
    sfe_otos_pose2d_t offset;
    QwiicOTOS otos;
};