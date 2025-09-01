#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <SCServo.h>
#include "Motor.h"

// Servo ID configuration
#define BASE_SERVO_COUNT 3
#define ARM_SERVO_COUNT 7  // 6 arm joints + 1 camera tilt

extern const uint8_t BASE_SERVO_IDS[BASE_SERVO_COUNT];
extern const uint8_t ARM_SERVO_IDS[ARM_SERVO_COUNT];

// Global servo objects (using SMS_STS library)
extern SMS_STS base_servo;
extern SMS_STS arm_servo;

// Global motor objects (using Motor wrapper)
extern Motor baseMotors[BASE_SERVO_COUNT];
extern Motor armMotors[ARM_SERVO_COUNT];

// Initialize servo communication
bool initServos(uint8_t rx_pin, uint8_t tx_pin);

// Initialize motor display (call after display is ready)
void initMotorDisplay(Adafruit_SSD1306* display);

// Check servo connectivity and configure
bool checkServos(Adafruit_SSD1306* display);

// Move arm to relaxed positions based on calibration
void moveToRelaxedPositions(Adafruit_SSD1306* display);

// Toggle servo control on/off
void toggleServoControl(bool enable, Adafruit_SSD1306* display);

// Control functions
void controlBaseServo(uint8_t servo_index, float rad_per_sec, bool servos_enabled);
void controlArmServo(uint8_t servo_index, float radians, bool servos_enabled);

// Emergency stop functions
void emergencyStopBase(Adafruit_SSD1306* display);
void emergencyStopArm(Adafruit_SSD1306* display);

// Update all motor states (call regularly in main loop)
void updateAllMotors();

#endif