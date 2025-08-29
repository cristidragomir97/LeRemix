#pragma once

#include <Arduino.h>
#include "config.h"

// Button manager class for handling EN button
class ButtonManager {
public:
    ButtonManager();
    void begin();
    void update();
    
    bool isPressed() const { return button_pressed_; }
    bool wasLongPressed() const { return long_press_detected_; }

private:
    bool button_pressed_;
    bool button_was_pressed_;
    bool long_press_detected_;
    unsigned long button_press_start_;
    
    void onLongPress();
};

extern ButtonManager g_button;