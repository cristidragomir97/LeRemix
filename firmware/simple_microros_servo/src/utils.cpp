#include "utils.h"
#include "config.h"

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