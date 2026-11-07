#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <cstdarg>
#include <string>
#include <vector>
#include <functional>
#include <algorithm>
#include <initializer_list>

inline uint32_t mockTime = 0;
inline int mockTransactions = 0;
inline uint32_t millis() { return mockTime; }
inline void delay(uint32_t ms) { mockTime += ms; }
inline void delayMicroseconds(uint32_t) {}
using portMUX_TYPE = int;
using TickType_t = uint32_t;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(lock) ((void)(lock))
#define portEXIT_CRITICAL(lock) ((void)(lock))
#define pdMS_TO_TICKS(ms) (ms)
inline TickType_t xTaskGetTickCount() { return mockTime; }
inline void vTaskDelayUntil(TickType_t* last, uint32_t period) { *last += period; mockTime = *last; }
inline void xTaskCreatePinnedToCore(void (*)(void*), const char*, int, void*, int, void*, int) {}
constexpr float RAD_TO_DEG = 57.2957795131f;
constexpr int HIGH = 1, LOW = 0, OUTPUT = 1, SERIAL_8N1 = 0;
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}

class Print {
public:
    std::string text;
    void print(const char* value) { text += value; }
    void println(const char* value) { text += std::string(value) + "\n"; }
    void printf(const char* format, ...) {
        char buffer[512];
        va_list args;
        va_start(args, format);
        std::vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        text += buffer;
    }
};
class HardwareSerial : public Print {
public:
    void begin(int, int, int, int) {}
    void flush() {}
    int available() { return 0; }
    int read() { return -1; }
};
inline HardwareSerial Serial, Serial1;
struct MockESP { void restart() {} };
inline MockESP ESP;
inline size_t strlcpy(char* out, const char* in, size_t capacity) {
    if (capacity) {
        const auto count = std::min(std::strlen(in), capacity - 1);
        std::memcpy(out, in, count);
        out[count] = '\0';
    }
    return std::strlen(in);
}
