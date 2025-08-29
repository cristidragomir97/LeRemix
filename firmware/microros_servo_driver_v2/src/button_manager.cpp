#include "button_manager.h"
#include "debug_utils.h"
#include "system_state.h"

ButtonManager g_button;

ButtonManager::ButtonManager() : 
    button_pressed_(false),
    button_was_pressed_(false),
    long_press_detected_(false),
    button_press_start_(0) {}

void ButtonManager::begin() {
    DEBUG_INFO("Initializing button manager");
    
    pinMode(EN_BUTTON_PIN, INPUT_PULLUP);
    
    DEBUG_INFO("Button manager initialized - long press to toggle servos");
}

void ButtonManager::update() {
    button_pressed_ = !digitalRead(EN_BUTTON_PIN);  // Button is active LOW
    
    if (button_pressed_ && !button_was_pressed_) {
        // Button just pressed
        button_press_start_ = millis();
        button_was_pressed_ = true;
        long_press_detected_ = false;
        DEBUG_INFO("Button press started");
    }
    else if (button_pressed_ && button_was_pressed_ && !long_press_detected_) {
        // Button held down - check for long press
        if (millis() - button_press_start_ >= LONG_PRESS_TIME) {
            long_press_detected_ = true;
            onLongPress();
        }
    }
    else if (!button_pressed_ && button_was_pressed_) {
        // Button just released
        if (!long_press_detected_) {
            DEBUG_INFO("Short button press (ignored)");
        }
        button_was_pressed_ = false;
        long_press_detected_ = false;
    }
}

void ButtonManager::onLongPress() {
    DEBUG_INFO("Long press detected - toggling servo control");
    g_system_state.toggleServos();
}