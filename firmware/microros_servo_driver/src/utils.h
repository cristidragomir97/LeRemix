#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>

// Debug and control defines
#define ENABLE_SERVO_CONTROL 1  // Set to 0 to disable servo functionality
#define ENABLE_DEBUG_PRINTS 1   // Set to 0 to disable debug prints

// Display functions
void displayMessage(const char* msg, Adafruit_SSD1306* display);
// void debugPrint(const char* msg);  // Deprecated - use displayMessage instead
void initScreen(Adafruit_SSD1306* display, uint8_t sda_pin, uint8_t scl_pin);

// Button handling
void checkButtonPress(uint8_t button_pin, bool* servos_enabled, 
                     unsigned long* button_press_start, bool* button_was_pressed, 
                     bool* button_long_press_detected, Adafruit_SSD1306* display);

// Servo conversion functions
float servoPositionToRadians(uint8_t servo_id, int32_t position);
int16_t radiansToServoPosition(uint8_t servo_id, float radians);

#endif