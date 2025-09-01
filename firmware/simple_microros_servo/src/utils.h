#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

// Debug and control defines
#define ENABLE_SERVO_CONTROL 1  // Set to 0 to disable servo functionality
#define ENABLE_DEBUG_PRINTS 1   // Set to 0 to disable debug prints

// Servo conversion functions
float servoPositionToRadians(uint8_t servo_id, int32_t position);
int16_t radiansToServoPosition(uint8_t servo_id, float radians);

#endif