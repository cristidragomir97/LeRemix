#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <STSServoDriver.h>

// Hardware configuration
extern const float VEL_TO_SERVO_UNIT; // Conversion factor from rad/s to servo speed units
extern const float COUNTS_PER_REV;     // Encoder counts per revolution

// Servo ID configuration
#define BASE_SERVO_COUNT 3
#define ARM_SERVO_COUNT 7  // 6 arm joints + 1 camera tilt

extern const uint8_t BASE_SERVO_IDS[BASE_SERVO_COUNT];
extern const uint8_t ARM_SERVO_IDS[ARM_SERVO_COUNT];

// Global servo objects
extern STSServoDriver base_servos;
extern STSServoDriver arm_servos;

// Initialize servo communication
bool initServos(uint8_t rx_pin, uint8_t tx_pin);

// Check servo connectivity and configure
bool checkServos(void* display);

// Move arm to relaxed positions based on calibration
void moveToRelaxedPositions(void* display);

// Control functions
void controlBaseServo(uint8_t servo_index, float rad_per_sec, bool servos_enabled);
void controlArmServo(uint8_t servo_index, float radians, bool servos_enabled);
void controlMultipleArmServos(float radians_array[ARM_SERVO_COUNT], bool servos_enabled);  // Batch arm servo control

// Emergency stop functions
void emergencyStopBase(void* display);
void emergencyStopArm(void* display);

// Breaking recipe function
void applyBreakingRecipe(uint8_t servo_id, STSServoDriver& servo_driver);

#endif