#include <Arduino.h>
#include <hw_config.h>
#include "comms.h"
#include "sensors.h"

static const uint32_t HEARTBEAT_INTERVAL_MS = 5000;
static uint32_t lastHeartbeat = 0;

#if ARS2_DEBUG
// Debug mode: INIT once the sensors are up, then print DATA periodically
static void debugPoll() {
    static bool initDone = false;
    static uint32_t lastData = 0;

    if (!sensorsReady()) return;

    if (!initDone) {
        initDone = true;
        commsRunLocal("INIT");
    }

    uint32_t now = millis();
    if (now - lastData >= DEBUG_DATA_PERIOD_MS) {
        lastData = now;
        commsRunLocal("DATA");
    }
}
#endif

void setup() {
    Serial.begin(BAUDRATE);
    delay(50);

    commsBegin();
    sensorsBegin();

    Serial.println("System Online.");
#if ARS2_DEBUG
    Serial.println("[DEBUG] debug mode on: packets are echoed to USB serial");
#endif
}

void loop() {
    commsPoll();
#if ARS2_DEBUG
    debugPoll();
#endif

    uint32_t now = millis();
    if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeat = now;
        Serial.println("[HEARTBEAT] system posted!");
    }

    delay(1);
}
