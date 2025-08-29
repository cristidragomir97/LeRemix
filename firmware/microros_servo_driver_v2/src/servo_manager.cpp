#include "servo_manager.h"
#include "debug_utils.h"
#include "system_state.h"

ServoManager g_servo;

ServoManager::ServoManager() : initialized_(false) {}

void ServoManager::begin() {
    DEBUG_INFO("Initializing servo manager");
    
    // Initialize serial communication for servos
    Serial2.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
    base_servo_.pSerial = &Serial2;
    arm_servo_.pSerial = &Serial2;
    
    if (checkServos()) {
        initializeBaseServos();
        initializeArmServos();
        
#if ENABLE_SERVO_CONTROL
        moveToRelaxedPositions();
#endif
        initialized_ = true;
        DEBUG_INFO("Servo manager initialized successfully");
    } else {
        DEBUG_ERROR("Servo initialization failed");
    }
}

void ServoManager::update() {
    // Servo manager updates are event-driven via command processing
}

bool ServoManager::checkServos() {
    DEBUG_INFO("Checking servo connectivity...");
    
    bool all_ok = true;
    
    // Check base servos
    for (int i = 0; i < BASE_SERVO_COUNT; i++) {
        if (!pingServo(base_servo_, BASE_SERVO_IDS[i], "BASE")) {
            all_ok = false;
        }
        delay(100);
    }
    
    // Check arm servos
    for (int i = 0; i < ARM_SERVO_COUNT; i++) {
        if (!pingServo(arm_servo_, ARM_SERVO_IDS[i], "ARM")) {
            all_ok = false;
        }
        delay(100);
    }
    
    return all_ok;
}

bool ServoManager::pingServo(SMS_STS& servo, uint8_t id, const char* name) {
    int pingStatus = servo.Ping(id);
    if (pingStatus == -1) {
        DEBUG_PRINTF("%s servo %d FAILED ping", name, id);
        return false;
    } else {
        DEBUG_PRINTF("%s servo %d OK", name, id);
        return true;
    }
}

void ServoManager::initializeBaseServos() {
    DEBUG_INFO("Initializing base servos (wheel mode)");
    
    for (int i = 0; i < BASE_SERVO_COUNT; i++) {
        uint8_t id = BASE_SERVO_IDS[i];
        base_servo_.WheelMode(id);      // Continuous rotation mode
        base_servo_.EnableTorque(id, 1); // Enable torque
        delay(50);
    }
}

void ServoManager::initializeArmServos() {
    DEBUG_INFO("Initializing arm servos (position mode)");
    
    for (int i = 0; i < ARM_SERVO_COUNT; i++) {
        uint8_t id = ARM_SERVO_IDS[i];
        // Servos should be in position mode by default
        arm_servo_.EnableTorque(id, 1);  // Enable torque
        delay(50);
    }
}

void ServoManager::moveToRelaxedPositions() {
    if (!initialized_) return;
    
    DEBUG_INFO("Moving arm to relaxed positions...");
    
    for (int i = 0; i < ARM_CALIBRATION_COUNT; i++) {
        const ServoCalibration& cal = ARM_CALIBRATION[i];
        arm_servo_.WritePosEx(cal.servo_id, cal.relaxed_position, 500, 0);
        delay(100);
        
        DEBUG_PRINTF("Servo %d -> position %d", cal.servo_id, cal.relaxed_position);
    }
    
    DEBUG_INFO("Relaxed position complete");
}

void ServoManager::stopAllServos() {
    if (!initialized_) return;
    
    DEBUG_INFO("Emergency stop - all servos");
    
    // Stop base servos
    for (int i = 0; i < BASE_SERVO_COUNT; i++) {
        base_servo_.WriteSpe(BASE_SERVO_IDS[i], 0, 255);  // Full brake
    }
    
    // Disable arm servo torque
    for (int i = 0; i < ARM_SERVO_COUNT; i++) {
        arm_servo_.EnableTorque(ARM_SERVO_IDS[i], 0);
    }
}

void ServoManager::enableAllServos() {
    if (!initialized_) return;
    
    DEBUG_INFO("Enabling all servos");
    
    // Re-enable arm servo torque
    for (int i = 0; i < ARM_SERVO_COUNT; i++) {
        arm_servo_.EnableTorque(ARM_SERVO_IDS[i], 1);
    }
}

void ServoManager::disableAllServos() {
    if (!initialized_) return;
    
    DEBUG_INFO("Disabling all servos");
    stopAllServos();
}

void ServoManager::processBaseCommand(const float* velocities, size_t count) {
    if (!initialized_ || !g_system_state.areServosEnabled()) return;
    
#if ENABLE_SERVO_CONTROL
    for (size_t i = 0; i < count && i < BASE_SERVO_COUNT; i++) {
        float rad_per_sec = velocities[i];
        
        if (abs(rad_per_sec) < 0.01) {  // Dead zone
            base_servo_.WriteSpe(BASE_SERVO_IDS[i], 0, 255);  // Brake
        } else {
            int16_t speed = constrain(rad_per_sec * VEL_TO_SERVO_UNIT, -4096, 4096);
            base_servo_.WriteSpe(BASE_SERVO_IDS[i], speed, 0);
        }
    }
#endif
}

void ServoManager::processArmCommand(const float* positions, size_t count) {
    if (!initialized_ || !g_system_state.areServosEnabled()) return;
    
#if ENABLE_SERVO_CONTROL
    for (size_t i = 0; i < count && i < ARM_SERVO_COUNT; i++) {
        float radians = positions[i];
        int16_t position = radiansToServoPosition(ARM_SERVO_IDS[i], radians);
        arm_servo_.WritePosEx(ARM_SERVO_IDS[i], position, 0, 0);
    }
#endif
}

bool ServoManager::readJointStates(float* positions, float* velocities, size_t count) {
    if (!initialized_) return false;
    
    // Read base servo states
    for (int i = 0; i < BASE_SERVO_COUNT && i < count; i++) {
        int32_t pos = base_servo_.ReadPos(BASE_SERVO_IDS[i]);
        if (pos != -1) {
            positions[i] = (pos * 2.0 * PI) / COUNTS_PER_REV;
        } else {
            positions[i] = 0.0;
        }
        
        int16_t speed = base_servo_.ReadSpeed(BASE_SERVO_IDS[i]);
        if (speed != -1) {
            velocities[i] = speed / VEL_TO_SERVO_UNIT;
        } else {
            velocities[i] = 0.0;
        }
    }
    
    // Read arm servo states
    for (int i = 0; i < ARM_SERVO_COUNT && (BASE_SERVO_COUNT + i) < count; i++) {
        int32_t pos = arm_servo_.ReadPos(ARM_SERVO_IDS[i]);
        if (pos != -1) {
            positions[BASE_SERVO_COUNT + i] = servoPositionToRadians(ARM_SERVO_IDS[i], pos);
        } else {
            positions[BASE_SERVO_COUNT + i] = 0.0;
        }
        
        int16_t speed = arm_servo_.ReadSpeed(ARM_SERVO_IDS[i]);
        if (speed != -1) {
            velocities[BASE_SERVO_COUNT + i] = (speed * 2.0 * PI) / (COUNTS_PER_REV * 60.0);
        } else {
            velocities[BASE_SERVO_COUNT + i] = 0.0;
        }
    }
    
    return true;
}

float ServoManager::servoPositionToRadians(uint8_t servo_id, int32_t position) {
    // Find calibration data for this servo
    for (int i = 0; i < ARM_CALIBRATION_COUNT; i++) {
        if (ARM_CALIBRATION[i].servo_id == servo_id) {
            const ServoCalibration& cal = ARM_CALIBRATION[i];
            // Map position from [min_pos, max_pos] to [-PI, PI] range
            float normalized = (float)(position - cal.min_position) / (float)(cal.max_position - cal.min_position);
            return (normalized * 2.0 * PI) - PI;
        }
    }
    // Fallback calculation
    return ((position - 2048) * 2.0 * PI) / COUNTS_PER_REV;
}

int16_t ServoManager::radiansToServoPosition(uint8_t servo_id, float radians) {
    // Find calibration data for this servo
    for (int i = 0; i < ARM_CALIBRATION_COUNT; i++) {
        if (ARM_CALIBRATION[i].servo_id == servo_id) {
            const ServoCalibration& cal = ARM_CALIBRATION[i];
            // Map radians from [-PI, PI] to [min_pos, max_pos] range
            float normalized = (radians + PI) / (2.0 * PI);
            int16_t position = cal.min_position + (int16_t)(normalized * (cal.max_position - cal.min_position));
            return constrain(position, cal.min_position, cal.max_position);
        }
    }
    // Fallback calculation
    return constrain(2048 + (radians * 651.74), 0, 4095);
}