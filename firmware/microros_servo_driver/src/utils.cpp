#include "utils.h"
#include "config.h"
#include <Wire.h>

#define LONG_PRESS_TIME 3000  // 3 seconds for long press

void displayMessage(const char* msg, Adafruit_SSD1306* display) {
  display->clearDisplay();
  display->setCursor(0, 0);
  display->print(msg);
  display->display();
  
  #if ENABLE_DEBUG_PRINTS
  Serial.print("[DISPLAY] ");
  Serial.println(msg);
  #endif
}

// Deprecated function - use displayMessage instead
// void debugPrint(const char* msg) {
//   #if ENABLE_DEBUG_PRINTS
//   Serial.print("[DEBUG] ");
//   Serial.println(msg);
//   #endif
// }

void initScreen(Adafruit_SSD1306* display, uint8_t sda_pin, uint8_t scl_pin) {
  Wire.begin(sda_pin, scl_pin);
  if(!display->begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display->clearDisplay();
  display->display();
  display->clearDisplay();
  display->setTextSize(1);
  display->setTextColor(SSD1306_WHITE);
  display->setCursor(0,0);
}

void checkButtonPress(uint8_t button_pin, bool* servos_enabled, 
                     unsigned long* button_press_start, bool* button_was_pressed, 
                     bool* button_long_press_detected, Adafruit_SSD1306* display) {
  bool button_pressed = !digitalRead(button_pin);  // Button is active LOW
  
  if (button_pressed && !*button_was_pressed) {
    // Button just pressed
    *button_press_start = millis();
    *button_was_pressed = true;
    *button_long_press_detected = false;
    displayMessage("Button press started", display);
  }
  else if (button_pressed && *button_was_pressed && !*button_long_press_detected) {
    // Button held down - check for long press
    if (millis() - *button_press_start >= LONG_PRESS_TIME) {
      *button_long_press_detected = true;
      displayMessage("Long press detected - toggling servo control", display);
      *servos_enabled = !*servos_enabled;
    }
  }
  else if (!button_pressed && *button_was_pressed) {
    // Button just released
    if (!*button_long_press_detected) {
      displayMessage("Short button press (ignored)", display);
    }
    *button_was_pressed = false;
    *button_long_press_detected = false;
  }
}

// Convert servo encoder position to radians using calibration data
float servoPositionToRadians(uint8_t servo_id, int32_t position) {
  // Find calibration data for this servo
  for (int i = 0; i < ARM_CALIBRATION_COUNT; i++) {
    if (ARM_CALIBRATION[i].servo_id == servo_id) {
      const ServoCalibration& cal = ARM_CALIBRATION[i];
      // Map position from [min_pos, max_pos] to [-PI, PI] range
      float normalized = (float)(position - cal.min_position) / (float)(cal.max_position - cal.min_position);
      return (normalized * 2.0 * PI) - PI; // Convert to -PI to PI range
    }
  }
  // Fallback to old calculation if calibration not found
  return ((position - 2048) * 2.0 * PI) / 4096.0;
}

// Convert radians to servo encoder position using calibration data
int16_t radiansToServoPosition(uint8_t servo_id, float radians) {
  // Find calibration data for this servo
  for (int i = 0; i < ARM_CALIBRATION_COUNT; i++) {
    if (ARM_CALIBRATION[i].servo_id == servo_id) {
      const ServoCalibration& cal = ARM_CALIBRATION[i];
      // Map radians from [-PI, PI] to [min_pos, max_pos] range
      float normalized = (radians + PI) / (2.0 * PI); // Convert to 0-1 range
      int16_t position = cal.min_position + (int16_t)(normalized * (cal.max_position - cal.min_position));
      return constrain(position, cal.min_position, cal.max_position);
    }
  }
  // Fallback to old calculation if calibration not found
  return constrain(2048 + (radians * 651.74), 0, 4095);
}