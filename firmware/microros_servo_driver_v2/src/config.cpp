#include "config.h"

// Servo ID configuration
const uint8_t BASE_SERVO_IDS[BASE_SERVO_COUNT] = {1, 2, 3};  // back, left, right motors
const uint8_t ARM_SERVO_IDS[ARM_SERVO_COUNT] = {4, 5, 6, 7, 8, 9, 10};  // joints 1-6 + camera_tilt

// Joint calibration data measured from physical robot
const ServoCalibration ARM_CALIBRATION[] = {
    {4, 1024, 3072, 2048},   // Joint 1 (pan): full left to full right, relaxed center
    {5, 880, 3072, 1024},    // Joint 2 (shoulder): full up to full down, relaxed down
    {6, 2024, 3880, 3880},   // Joint 3 (elbow): full up to full down, relaxed down
    {7, 1024, 3072, 1024},   // Joint 4 (wrist tilt): full up to full down, relaxed up
    {8, 1372, 4096, 2048},   // Joint 5 (wrist rotation): full left to full right, relaxed center
    {9, 2048, 3446, 2048},   // Joint 6 (gripper): closed to open, relaxed center
    {10, 1024, 2611, 2048}   // Camera tilt: full down to full up, relaxed center
};

const int ARM_CALIBRATION_COUNT = sizeof(ARM_CALIBRATION) / sizeof(ServoCalibration);