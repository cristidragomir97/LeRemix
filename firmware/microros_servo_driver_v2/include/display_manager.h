#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "config.h"

// Display manager class for OLED screen
class DisplayManager {
public:
    DisplayManager();
    void begin();
    void update();
    void displayMessage(const char* message);
    void displayStatus();
    void clear();

private:
    Adafruit_SSD1306 display_;
    String current_message_;
    unsigned long last_update_;
    
    void initializeDisplay();
};

extern DisplayManager g_display;