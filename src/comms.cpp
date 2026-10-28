#include "comms.h"
#include <Arduino.h>
#include <hw_config.h>
#include "sensors.h"

namespace {

HardwareSerial& bus = Serial1;

char rxBuf[121];
size_t rxLen = 0;
bool inPacket = false;

// Commands run locally (debug mode) reply to USB serial instead of the bus
bool localCmd = false;

void beginTx() {
    if (localCmd) return;
    bus.flush();
    digitalWrite(RS485_EN_PIN, HIGH);
    delayMicroseconds(50); // transceiver settle
}

void endTx() {
    if (localCmd) return;
    bus.flush(); // waits until the last byte has left the wire
    delayMicroseconds(30);
    digitalWrite(RS485_EN_PIN, LOW);
}

void writeLine(const char* s) {
    if (localCmd) {
        Serial.printf("[LOCAL TX] %s\n", s);
        return;
    }
    bus.print(s);
    bus.print("\r\n");
    if (ARS2_DEBUG) Serial.printf("[TX] %s\n", s);
}

void reply(const char* s) {
    beginTx();
    writeLine(s);
    endTx();
}

void sendData() {
    SensorReading r[SENSOR_COUNT];
    sensorsSnapshot(r);

    char line[96];
    beginTx();
    writeLine("<ACK><DATA>");
    for (const SensorReading& s : r) {
        if (!s.valid) continue;
        snprintf(line, sizeof(line), "%s(%+ld, %+ld, %+ld)<$>",
                 s.name, (long)s.a, (long)s.b, (long)s.c);
        writeLine(line);
    }
    writeLine("<EOL>");
    endTx();
}

void handleCommand(char* cmd) {
    if (*cmd == '\0') return;

    for (char* p = cmd; *p; p++) *p = toupper((unsigned char)*p);

    if (ARS2_DEBUG) Serial.printf("[%s] #%s\n", localCmd ? "LOCAL RX" : "RX", cmd);

    if (strstr(cmd, "DATA")) {
        sendData();
        return;
    }

    if (!ARS2_DEBUG) Serial.printf("Received: %s\n", cmd);

    if (strstr(cmd, "<OFFS>")) {
        reply("<ACK><OFFS>(OK)<EOL>");
        return;
    }

    if (strstr(cmd, "HRST") || strstr(cmd, "SRST")) {
        ESP.restart();
    }

    if (strstr(cmd, "INIT")) {
        char resp[48];
        snprintf(resp, sizeof(resp), "<ACK><INIT>(ADDED=%u,BAD=0)<EOL>", (unsigned)SENSOR_COUNT);
        reply(resp);
        return;
    }

    if (strstr(cmd, "PING")) {
        reply("<ACK><UNKO>(PONG-PONG)<EOL>");
        return;
    }

    reply("<ACK><ERRR>(UNKNOWN)<EOL>");
}

} // namespace

void commsBegin() {
    pinMode(RS485_EN_PIN, OUTPUT);
    digitalWrite(RS485_EN_PIN, LOW); // idle in receive mode
    bus.begin(BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
}

void commsRunLocal(const char* cmd) {
    char buf[sizeof(rxBuf)];
    strlcpy(buf, cmd, sizeof(buf));
    localCmd = true;
    handleCommand(buf);
    localCmd = false;
}

void commsPoll() {
    while (bus.available() > 0) {
        char c = (char)bus.read();

        if (!inPacket) {
            if (c == '#') {
                inPacket = true;
                rxLen = 0;
            }
            continue;
        }

        if (c == '\n') {
            rxBuf[rxLen] = '\0';
            inPacket = false;
            handleCommand(rxBuf);
            continue;
        }

        if ((uint8_t)c < 32 || (uint8_t)c > 126) continue; // drops '\r' and line noise

        if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = c;
        else inPacket = false; // too long, discard
    }
}
