#pragma once

#include <Arduino.h>
#include <SCServo.h>
#include "config.h"

// Servo manager class for handling all servo operations
class ServoManager {
public:
    ServoManager();
    void begin();
    void update();
    
    // Servo control functions
    bool checkServos();
    void moveToRelaxedPositions();
    void stopAllServos();
    void enableAllServos();
    void disableAllServos();
    
    // Command processing
    void processBaseCommand(const float* velocities, size_t count);
    void processArmCommand(const float* positions, size_t count);
    
    // State reading
    bool readJointStates(float* positions, float* velocities, size_t count);
    
    // Conversion functions
    static float servoPositionToRadians(uint8_t servo_id, int32_t position);
    static int16_t radiansToServoPosition(uint8_t servo_id, float radians);

private:
    SMS_STS base_servo_;
    SMS_STS arm_servo_;
    bool initialized_;
    
    // Helper functions
    bool pingServo(SMS_STS& servo, uint8_t id, const char* name);
    void initializeBaseServos();
    void initializeArmServos();
};

extern ServoManager g_servo;