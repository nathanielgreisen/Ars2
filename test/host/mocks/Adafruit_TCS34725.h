#pragma once
#include "Arduino.h"
constexpr int TCS34725_INTEGRATIONTIME_2_4MS = 0, TCS34725_GAIN_60X = 0;
class Adafruit_TCS34725 {
public:
    Adafruit_TCS34725(int, int) {}
    bool begin() { return true; }
    void getRawData(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c) { *r = *g = *b = *c = 0; }
};
