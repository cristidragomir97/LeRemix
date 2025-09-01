#include "motor_controller.h"
#include "utils.h"
#include "config.h"
#include "debug_serial.h"

// Hardware configuration
const float VEL_TO_SERVO_UNIT = 4096; // Conversion factor from rad/s to servo speed units (4096 counts/rev)
const float COUNTS_PER_REV = 4096;     // Encoder counts per revolution

// Servo ID configuration
const uint8_t BASE_SERVO_IDS[BASE_SERVO_COUNT] = {3, 2, 1};  // back, left, right motors
const uint8_t ARM_SERVO_IDS[ARM_SERVO_COUNT] = {4, 5, 6, 7, 8, 9, 11};  // joints 1-6 + camera_tilt

// Global servo objects
STSServoDriver base_servos;
STSServoDriver arm_servos;

// Track current modes to avoid unnecessary mode switches
static STSMode base_servo_modes[BASE_SERVO_COUNT] = {STSMode::VELOCITY, STSMode::VELOCITY, STSMode::VELOCITY};


bool initServos(uint8_t rx_pin, uint8_t tx_pin) {
    Serial2.begin(1000000, SERIAL_8N1, rx_pin, tx_pin);
    
    bool base_init = base_servos.init(&Serial2);
    bool arm_init = arm_servos.init(&Serial2);
    
    return base_init && arm_init;
}

bool checkServos(void* display) {
    Serial1.println("Checking base servos...");
    delay(50);

    // Check base servos (wheel motors)
    for (int i = 0; i < BASE_SERVO_COUNT; i++) {
        uint8_t servo_id = BASE_SERVO_IDS[i];
        Serial.printf("Pinging base servo %d...\n", servo_id);
        bool pingStatus = base_servos.ping(servo_id);
        if (!pingStatus) {
            Serial.printf("Base servo %d FAIL!\n", servo_id);
            delay(200);
        } else {
            Serial.printf("Base servo %d ping OK, setting up...\n", servo_id);
            base_servos.setMode(servo_id, STSMode::VELOCITY);  // Set to velocity mode
            delay(50); // Critical delay after mode change
            base_servos.setTorqueEnable(servo_id, true);
            delay(50); // Delay after torque enable
            Serial.printf("Base servo %d SETUP COMPLETE\n", servo_id);
        }
    }

    Serial1.println("Checking arm servos...");
    delay(50);

    // Check arm servos (position control)
    for (int i = 0; i < ARM_SERVO_COUNT; i++) {
        uint8_t servo_id = ARM_SERVO_IDS[i];
        bool pingStatus = arm_servos.ping(servo_id);
        if (!pingStatus) {
            Serial.printf("Arm servo %d FAIL!\n", servo_id);
            delay(100);
        } else {
            arm_servos.setMode(servo_id, STSMode::POSITION);  // Set to position mode
            arm_servos.setTorqueEnable(servo_id, true);
            Serial.printf("Arm servo %d OK\n", servo_id);
            delay(50);
        }
    }

    Serial1.println("All servos OK!");
    delay(100);
    return true;
}

void moveToRelaxedPositions(void* display) {
    Serial1.println("Moving to relaxed positions...");
    
    // Move arm servos to their calibrated relaxed positions
    for (int i = 0; i < ARM_CALIBRATION_COUNT; i++) {
        const ServoCalibration& cal = ARM_CALIBRATION[i];
        arm_servos.setTargetPosition(cal.servo_id, cal.relaxed_position);
        delay(50); // Small delay between commands
        
        Serial.printf("Servo %d -> %d\n", cal.servo_id, cal.relaxed_position);
        delay(100);
    }
    
    Serial1.println("Relaxed position set!");
    delay(500);
}


void controlBaseServo(uint8_t servo_index, float rad_per_sec, bool servos_enabled) {
    if (servo_index >= BASE_SERVO_COUNT) return;
    
    uint8_t servo_id = BASE_SERVO_IDS[servo_index];
    
    #if ENABLE_DEBUG_PRINTS
    Serial1.printf("Servo %d (ID=%d): %.4f rad/s", servo_index, servo_id, rad_per_sec);
    #endif
    
    #if ENABLE_SERVO_CONTROL
    if (servos_enabled) {
        if (abs(rad_per_sec) < 0.1) {  // Dead zone - apply position brake
            // Read current position and lock there (like servo_test_sketch)
            int16_t currentPos = base_servos.getCurrentPosition(servo_id);
            if (currentPos != -1) {  // Valid position read
                // Switch to position mode only if not already in position mode
                if (base_servo_modes[servo_index] != STSMode::POSITION) {
                    base_servos.setMode(servo_id, STSMode::POSITION);
                    delayMicroseconds(50); // Increased delay for mode switch - critical for STS servos
                    base_servo_modes[servo_index] = STSMode::POSITION;
                }
                base_servos.setTargetPosition(servo_id, currentPos);
                #if ENABLE_DEBUG_PRINTS
                Serial.printf(" -> POSITION_BRAKE at %d\n", currentPos);
                #endif
            } else {
                // Fallback to velocity brake if position read fails
                if (base_servo_modes[servo_index] != STSMode::VELOCITY) {
                    base_servos.setMode(servo_id, STSMode::VELOCITY);
                    delayMicroseconds(50);
                    base_servo_modes[servo_index] = STSMode::VELOCITY;
                }
                base_servos.setTargetVelocity(servo_id, 0);
                #if ENABLE_DEBUG_PRINTS
                Serial.printf(" -> VELOCITY_BRAKE (pos read fail)\n");
                #endif
            }
        } else {
            // Ensure we're in velocity mode for movement
            if (base_servo_modes[servo_index] != STSMode::VELOCITY) {
                base_servos.setMode(servo_id, STSMode::VELOCITY);
                delayMicroseconds(50); // Delay for mode switch - critical for STS servos
                base_servo_modes[servo_index] = STSMode::VELOCITY;
            }
            
            // Normal speed command
            int16_t speed = constrain(rad_per_sec * VEL_TO_SERVO_UNIT, -4096, 4096);
            base_servos.setTargetVelocity(servo_id, speed);
            DEBUG_MOTOR_SENT("BASE", servo_id, speed);
            #if ENABLE_DEBUG_PRINTS
            Serial.printf(" -> SPEED: %d\n", speed);
            #endif
        }
    } else {
        #if ENABLE_DEBUG_PRINTS
        Serial.printf(" -> DISABLED\n");
        #endif
    }
    #else
    #if ENABLE_DEBUG_PRINTS
    Serial.printf(" -> SERVO_CONTROL_DISABLED\n");
    #endif
    #endif
}


void controlArmServo(uint8_t servo_index, float radians, bool servos_enabled) {
    if (servo_index >= ARM_SERVO_COUNT) return;
    
    uint8_t servo_id = ARM_SERVO_IDS[servo_index];
    DEBUG_MOTOR_COMMAND("ARM", servo_id, radians);
    
    #if ENABLE_DEBUG_PRINTS
    Serial.printf("Servo %d (ID=%d): %.4f rad", servo_index, servo_id, radians);
    #endif
    
    #if ENABLE_SERVO_CONTROL
    if (servos_enabled) {
        // Use calibrated conversion function
        int16_t position = radiansToServoPosition(servo_id, radians);
        arm_servos.setTargetPosition(servo_id, position);
        DEBUG_MOTOR_SENT("ARM", servo_id, position);
        #if ENABLE_DEBUG_PRINTS
        Serial.printf(" -> POS: %d\n", position);
        #endif
    } else {
        #if ENABLE_DEBUG_PRINTS
        Serial.printf(" -> DISABLED\n");
        #endif
    }
    #else
    #if ENABLE_DEBUG_PRINTS
    Serial.printf(" -> SERVO_CONTROL_DISABLED\n");
    #endif
    #endif
}

void controlMultipleArmServos(float radians_array[ARM_SERVO_COUNT], bool servos_enabled) {
    #if ENABLE_DEBUG_PRINTS
    Serial1.println("=== MULTIPLE ARM SERVO COMMAND ===");
    #endif
    
    #if ENABLE_SERVO_CONTROL
    if (servos_enabled) {
        // Convert all radians to servo positions first
        int16_t positions[ARM_SERVO_COUNT];
        
        for (int i = 0; i < ARM_SERVO_COUNT; i++) {
            uint8_t servo_id = ARM_SERVO_IDS[i];
            positions[i] = radiansToServoPosition(servo_id, radians_array[i]);
            
            #if ENABLE_DEBUG_PRINTS
            Serial.printf("Servo %d: %.4f rad -> %d pos\n", servo_id, radians_array[i], positions[i]);
            #endif
        }
        
        // Send all position commands in rapid succession (batched)
        for (int i = 0; i < ARM_SERVO_COUNT; i++) {
            uint8_t servo_id = ARM_SERVO_IDS[i];
            arm_servos.setTargetPosition(servo_id, positions[i]);
            DEBUG_MOTOR_SENT("ARM", servo_id, positions[i]);
        }
        
        #if ENABLE_DEBUG_PRINTS
        Serial1.println("Multiple arm commands sent!");
        #endif
    } else {
        #if ENABLE_DEBUG_PRINTS
        Serial1.println("ARM servos disabled - skipping multiple command");
        #endif
    }
    #else
    #if ENABLE_DEBUG_PRINTS
    Serial1.println("SERVO_CONTROL_DISABLED - skipping multiple command");
    #endif
    #endif
}

void emergencyStopBase(void* display) {
    for (int i = 0; i < BASE_SERVO_COUNT; i++) {
        base_servos.setTargetVelocity(BASE_SERVO_IDS[i], 0);  // Stop velocity
        base_servos.setTorqueEnable(BASE_SERVO_IDS[i], false);  // Disable torque
    }
    Serial1.println("Base emergency stop activated!");
}

void emergencyStopArm(void* display) {
    for (int i = 0; i < ARM_SERVO_COUNT; i++) {
        arm_servos.setTorqueEnable(ARM_SERVO_IDS[i], false);  // Disable torque
    }
    Serial1.println("Arm emergency stop activated!");
}