#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <SCServo.h>
#include <Adafruit_SSD1306.h>

/**
 * @brief Motor class - Encapsulates a single servo motor with state management
 * 
 * This class wraps a servo and provides high-level control with automatic
 * braking using position mode when speed is set to 0.
 */
class Motor {
private:
  uint8_t id;                    // Servo ID
  SMS_STS* servo;                // Pointer to servo object
  int16_t lastPosition;          // Last known position
  float currentSpeed;            // Current commanded speed (rad/s)
  bool isMoving;                 // Movement state
  unsigned long lastUpdateTime;  // Last update timestamp
  bool isBaseMotor;              // true for base motors, false for arm motors
  static Adafruit_SSD1306* display; // Shared display pointer for all motors
  
  // Constants for conversion
  static constexpr float SERVO_COUNTS_PER_REV = 4096.0f;
  static constexpr float RAD_PER_REV = 2.0f * PI;
  
public:
  /**
   * @brief Constructor
   * @param servoId Servo ID
   * @param servoObject Pointer to the SMS_STS servo object
   * @param isBase true if this is a base motor, false for arm motor
   */
  Motor(uint8_t servoId, SMS_STS* servoObject, bool isBase = false);
  
  /**
   * @brief Initialize the motor
   * @return true if successful, false otherwise
   */
  bool init();
  
  /**
   * @brief Set motor speed
   * @param rad_per_sec Speed in rad/s (0 = brake using position mode)
   * @return true if successful, false otherwise
   */
  bool setSpeed(float rad_per_sec);
  
  /**
   * @brief Update motor state (call regularly in main loop)
   */
  void update();
  
  /**
   * @brief Set motor to position mode and move to target
   * @param position Target position (0-4095)
   * @return true if successful, false otherwise
   */
  bool setPosition(int16_t position);
  
  /**
   * @brief Enable or disable motor torque
   * @param enable true to enable, false to disable
   * @return true if successful, false otherwise
   */
  bool setTorqueEnable(bool enable);
  
  /**
   * @brief Get current position
   * @return Current position or -1 if error
   */
  int16_t getCurrentPosition();
  
  /**
   * @brief Get current velocity from servo
   * @return Current velocity in servo units or -1 if error
   */
  int16_t getCurrentVelocity();
  
  /**
   * @brief Check if motor is currently moving
   * @return true if moving, false if stopped
   */
  bool getIsMoving() const { return isMoving; }
  
  /**
   * @brief Get current commanded speed
   * @return Current speed in rad/s
   */
  float getCurrentSpeed() const { return currentSpeed; }
  
  /**
   * @brief Get last known position
   * @return Last position
   */
  int16_t getLastPosition() const { return lastPosition; }
  
  /**
   * @brief Get servo ID
   * @return Servo ID
   */
  uint8_t getId() const { return id; }
  
  /**
   * @brief Ping the servo
   * @return true if servo responds, false otherwise
   */
  bool ping();
  
  /**
   * @brief Set display pointer for all motors
   * @param displayPtr Pointer to the display object
   */
  static void setDisplay(Adafruit_SSD1306* displayPtr);
  
private:
  /**
   * @brief Convert rad/s to servo speed units
   * @param rad_per_sec Speed in rad/s
   * @return Speed in servo units
   */
  int16_t radPerSecToServoSpeed(float rad_per_sec);
  
  /**
   * @brief Convert servo speed units to rad/s
   * @param servo_speed Speed in servo units
   * @return Speed in rad/s
   */
  float servoSpeedToRadPerSec(int16_t servo_speed);
};

#endif // MOTOR_H