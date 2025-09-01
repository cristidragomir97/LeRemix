#ifndef STSSERVODRIVER_H
#define STSSERVODRIVER_H

#include <Arduino.h>
#include <HardwareSerial.h>

// Servo operation modes
enum class STSMode {
  POSITION,
  VELOCITY
};

/**
 * @brief STSServoDriver - A wrapper class for STS servo control
 * 
 * This class provides a high-level interface for controlling STS servos,
 * supporting both position and velocity control modes with proper encapsulation.
 */
class STSServoDriver {
private:
  HardwareSerial* serial;
  bool initialized;
  
  // Low-level servo communication functions
  bool writeInstruction(uint8_t id, uint8_t instruction, uint8_t* params, uint8_t paramLength);
  bool readResponse(uint8_t id, uint8_t* buffer, uint8_t expectedLength);
  uint8_t calculateChecksum(uint8_t* data, uint8_t length);
  
  // STS protocol constants
  static const uint8_t STS_HEADER = 0xFF;
  static const uint8_t STS_BROADCAST_ID = 0xFE;
  
  // STS instruction set
  static const uint8_t INST_PING = 0x01;
  static const uint8_t INST_READ = 0x02;
  static const uint8_t INST_WRITE = 0x03;
  static const uint8_t INST_WRITE_ACC = 0x04;
  static const uint8_t INST_WRITE_TIME = 0x05;
  static const uint8_t INST_WRITE_TIME_ACC = 0x06;
  
  // STS register addresses (from SMS_STS.h)
  static const uint8_t REG_MODE = 33;                    // SMS_STS_MODE
  static const uint8_t REG_TORQUE_ENABLE = 40;          // SMS_STS_TORQUE_ENABLE
  static const uint8_t REG_TARGET_POSITION_L = 42;      // SMS_STS_GOAL_POSITION_L
  static const uint8_t REG_TARGET_POSITION_H = 43;      // SMS_STS_GOAL_POSITION_H
  static const uint8_t REG_TARGET_VELOCITY_L = 46;      // SMS_STS_GOAL_SPEED_L
  static const uint8_t REG_TARGET_VELOCITY_H = 47;      // SMS_STS_GOAL_SPEED_H
  static const uint8_t REG_PRESENT_POSITION_L = 56;     // SMS_STS_PRESENT_POSITION_L
  static const uint8_t REG_PRESENT_POSITION_H = 57;     // SMS_STS_PRESENT_POSITION_H
  static const uint8_t REG_PRESENT_VELOCITY_L = 58;     // SMS_STS_PRESENT_SPEED_L
  static const uint8_t REG_PRESENT_VELOCITY_H = 59;     // SMS_STS_PRESENT_SPEED_H
  static const uint8_t REG_MOVING = 66;                 // SMS_STS_MOVING
  
  // Mode values
  static const uint8_t MODE_POSITION = 0x00;
  static const uint8_t MODE_VELOCITY = 0x01;

public:
  /**
   * @brief Constructor
   */
  STSServoDriver();
  
  /**
   * @brief Destructor
   */
  ~STSServoDriver();
  
  /**
   * @brief Initialize the servo driver with a serial port
   * @param serialPort Pointer to the HardwareSerial to use
   * @return true if initialization successful, false otherwise
   */
  bool init(HardwareSerial* serialPort);
  
  /**
   * @brief Ping a servo to check if it's responding
   * @param id Servo ID to ping
   * @return true if servo responds, false otherwise
   */
  bool ping(uint8_t id);
  
  /**
   * @brief Set the operating mode of a servo
   * @param id Servo ID
   * @param mode Operating mode (POSITION or VELOCITY)
   * @return true if successful, false otherwise
   */
  bool setMode(uint8_t id, STSMode mode);
  
  /**
   * @brief Enable or disable torque for a servo
   * @param id Servo ID
   * @param enable true to enable torque, false to disable
   * @return true if successful, false otherwise
   */
  bool setTorqueEnable(uint8_t id, bool enable);
  
  /**
   * @brief Set target position for a servo (position mode)
   * @param id Servo ID
   * @param position Target position (0-4095)
   * @return true if successful, false otherwise
   */
  bool setTargetPosition(uint8_t id, int16_t position);
  
  /**
   * @brief Set target velocity for a servo (velocity mode)
   * @param id Servo ID
   * @param velocity Target velocity in servo units
   * @return true if successful, false otherwise
   */
  bool setTargetVelocity(uint8_t id, int16_t velocity);
  
  /**
   * @brief Get current position of a servo
   * @param id Servo ID
   * @return Current position (0-4095) or -1 if error
   */
  int16_t getCurrentPosition(uint8_t id);
  
  /**
   * @brief Get current velocity of a servo
   * @param id Servo ID
   * @return Current velocity in servo units or -1 if error
   */
  int16_t getCurrentVelocity(uint8_t id);
  
  /**
   * @brief Check if a servo is currently moving
   * @param id Servo ID
   * @return true if moving, false if stopped or error
   */
  bool isMoving(uint8_t id);
  
  /**
   * @brief Check if driver is initialized
   * @return true if initialized, false otherwise
   */
  bool isInitialized() const { return initialized; }
};

#endif // STSSERVODRIVER_H