#include <Arduino.h>
#include <hw_config.h>
#include "comms.h"
#include "sensors.h"

static const uint32_t HEARTBEAT_INTERVAL_MS = 5000;
static uint32_t lastHeartbeat = 0;

void setup() {
    Serial.begin(BAUDRATE);
    delay(50);

    commsBegin();
    sensorsBegin();

    Serial.println("System Online.");
}

void loop() {
    commsPoll();

    uint32_t now = millis();
    if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeat = now;
        Serial.println("[HEARTBEAT] system posted!");
    }

    delay(1);
}
