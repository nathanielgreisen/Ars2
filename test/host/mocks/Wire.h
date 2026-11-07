#pragma once
#include "Arduino.h"

inline bool mockPinMissing = false, mockPinNeverReady = false, mockShortRead = false;
inline uint32_t mockPinResetAt = 0;
inline unsigned mockPinResets = 0;
inline uint8_t mockSelectedPort = 0;

class TwoWire {
    uint8_t address = 0, reg = 0;
    std::vector<uint8_t> written;
    uint8_t received[40] = {};
public:
    void begin(int, int, int) {}
    void beginTransmission(uint8_t value) { address = value; written.clear(); ++mockTransactions; }
    void write(uint8_t value) { written.push_back(value); }
    void write(const uint8_t* data, size_t size) { written.insert(written.end(), data, data + size); }
    int endTransmission() {
        if (address == 0x70 && !written.empty()) {
            for (int i = 0; i < 8; ++i) if (written[0] == (1u << i)) mockSelectedPort = i;
        }
        if (address == 0x31) {
            if (mockPinMissing) return 1;
            reg = written[0];
            if (reg == 4 && written.size() == 5) {
                uint32_t control;
                std::memcpy(&control, written.data() + 1, 4);
                if (control == 2) { mockPinResetAt = millis(); ++mockPinResets; }
            }
        }
        return 0;
    }
    size_t requestFrom(uint8_t, size_t length) {
        std::memset(received, 0, sizeof(received));
        uint32_t value = 1;
        if (reg == 18) value = mockPinNeverReady || millis() - mockPinResetAt < 250 ? 2 : 1;
        std::memcpy(received, &value, 4);
        return length;
    }
    size_t readBytes(uint8_t* out, size_t length) {
        const auto count = mockShortRead ? length - 1 : length;
        std::memcpy(out, received, count);
        return count;
    }
};
inline TwoWire Wire;
