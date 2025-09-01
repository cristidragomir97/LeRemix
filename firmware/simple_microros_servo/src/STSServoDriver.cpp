#include "STSServoDriver.h"

STSServoDriver::STSServoDriver() : serial(nullptr), initialized(false) {
}

STSServoDriver::~STSServoDriver() {
  // Nothing to clean up
}

bool STSServoDriver::init(HardwareSerial* serialPort) {
  if (serialPort == nullptr) {
    return false;
  }
  
  serial = serialPort;
  initialized = true;
  
  return true;
}

bool STSServoDriver::writeInstruction(uint8_t id, uint8_t instruction, uint8_t* params, uint8_t paramLength) {
  if (!initialized || serial == nullptr) {
    return false;
  }
  
  uint8_t packetLength = 6 + paramLength; // Header(2) + ID(1) + Length(1) + Instruction(1) + Params(N) + Checksum(1)
  uint8_t packet[packetLength];
  
  // Build packet
  packet[0] = STS_HEADER;
  packet[1] = STS_HEADER;
  packet[2] = id;
  packet[3] = paramLength + 2; // Length = instruction + params + checksum
  packet[4] = instruction;
  
  // Copy parameters
  if (params != nullptr && paramLength > 0) {
    memcpy(&packet[5], params, paramLength);
  }
  
  // Calculate and add checksum
  packet[packetLength - 1] = calculateChecksum(&packet[2], packetLength - 3);
  
  // Send packet
  serial->write(packet, packetLength);
  serial->flush();
  
  return true;
}

bool STSServoDriver::readResponse(uint8_t id, uint8_t* buffer, uint8_t expectedLength) {
  if (!initialized || serial == nullptr || buffer == nullptr) {
    return false;
  }
  
  unsigned long timeout = millis() + 100; // 100ms timeout
  uint8_t index = 0;
  
  while (millis() < timeout && index < expectedLength) {
    if (serial->available()) {
      buffer[index++] = serial->read();
    }
    yield(); // Prevent watchdog timeout
  }
  
  // Check if we received the expected amount of data
  if (index < expectedLength) {
    return false;
  }
  
  // Verify packet structure
  if (buffer[0] != STS_HEADER || buffer[1] != STS_HEADER) {
    return false;
  }
  
  // Verify ID
  if (buffer[2] != id) {
    return false;
  }
  
  // Verify checksum
  uint8_t receivedChecksum = buffer[expectedLength - 1];
  uint8_t calculatedChecksum = calculateChecksum(&buffer[2], expectedLength - 3);
  
  return (receivedChecksum == calculatedChecksum);
}

uint8_t STSServoDriver::calculateChecksum(uint8_t* data, uint8_t length) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < length; i++) {
    sum += data[i];
  }
  return ~sum; // Bitwise NOT
}

bool STSServoDriver::ping(uint8_t id) {
  if (!writeInstruction(id, INST_PING, nullptr, 0)) {
    return false;
  }
  
  uint8_t response[6]; // Expected response: FF FF ID 02 00 Checksum
  return readResponse(id, response, 6);
}

bool STSServoDriver::setMode(uint8_t id, STSMode mode) {
  uint8_t modeValue = (mode == STSMode::POSITION) ? MODE_POSITION : MODE_VELOCITY;
  uint8_t params[2] = {REG_MODE, modeValue};
  
  return writeInstruction(id, INST_WRITE, params, 2);
}

bool STSServoDriver::setTorqueEnable(uint8_t id, bool enable) {
  uint8_t torqueValue = enable ? 1 : 0;
  uint8_t params[2] = {REG_TORQUE_ENABLE, torqueValue};
  
  return writeInstruction(id, INST_WRITE, params, 2);
}

bool STSServoDriver::setTargetPosition(uint8_t id, int16_t position) {
  // Clamp position to valid range
  if (position < 0) position = 0;
  if (position > 4095) position = 4095;
  
  uint8_t params[3] = {
    REG_TARGET_POSITION_L,
    (uint8_t)(position & 0xFF),        // Low byte
    (uint8_t)((position >> 8) & 0xFF)  // High byte
  };
  
  return writeInstruction(id, INST_WRITE, params, 3);
}

bool STSServoDriver::setTargetVelocity(uint8_t id, int16_t velocity) {
  // Handle direction - negative velocities
  uint16_t velocityValue;
  if (velocity < 0) {
    velocityValue = (uint16_t)(-velocity) | 0x8000; // Set direction bit
  } else {
    velocityValue = (uint16_t)velocity;
  }
  
  uint8_t params[3] = {
    REG_TARGET_VELOCITY_L,
    (uint8_t)(velocityValue & 0xFF),        // Low byte
    (uint8_t)((velocityValue >> 8) & 0xFF)  // High byte
  };
  
  return writeInstruction(id, INST_WRITE, params, 3);
}

int16_t STSServoDriver::getCurrentPosition(uint8_t id) {
  uint8_t params[2] = {REG_PRESENT_POSITION_L, 2}; // Read 2 bytes from position register
  
  if (!writeInstruction(id, INST_READ, params, 2)) {
    return -1;
  }
  
  uint8_t response[8]; // FF FF ID 04 00 POS_L POS_H Checksum
  if (!readResponse(id, response, 8)) {
    return -1;
  }
  
  // Extract position from response
  int16_t position = (response[6] << 8) | response[5]; // High byte << 8 | Low byte
  return position;
}

int16_t STSServoDriver::getCurrentVelocity(uint8_t id) {
  uint8_t params[2] = {REG_PRESENT_VELOCITY_L, 2}; // Read 2 bytes from velocity register
  
  if (!writeInstruction(id, INST_READ, params, 2)) {
    return -1;
  }
  
  uint8_t response[8]; // FF FF ID 04 00 VEL_L VEL_H Checksum
  if (!readResponse(id, response, 8)) {
    return -1;
  }
  
  // Extract velocity from response
  uint16_t velocityRaw = (response[6] << 8) | response[5]; // High byte << 8 | Low byte
  
  // Handle direction bit
  if (velocityRaw & 0x8000) {
    // Negative direction
    return -((int16_t)(velocityRaw & 0x7FFF));
  } else {
    // Positive direction
    return (int16_t)velocityRaw;
  }
}

bool STSServoDriver::isMoving(uint8_t id) {
  uint8_t params[2] = {REG_MOVING, 1}; // Read 1 byte from moving register
  
  if (!writeInstruction(id, INST_READ, params, 2)) {
    return false;
  }
  
  uint8_t response[7]; // FF FF ID 03 00 MOVING Checksum
  if (!readResponse(id, response, 7)) {
    return false;
  }
  
  return (response[5] != 0); // Non-zero means moving
}