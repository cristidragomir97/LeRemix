#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"
#include "system_state.h"

// NeoPixel status manager class
class NeoPixelManager {
public:
    NeoPixelManager();
    void begin();
    void update();
    void updateStatus(SystemState state);
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    void setBrightness(uint8_t brightness);
    void clear();

private:
    Adafruit_NeoPixel pixels_;
    SystemState last_state_;
    
    uint32_t getColorForState(SystemState state);
};

extern NeoPixelManager g_neopixel;