#include "motor_controller.h"
#include "utils.h"
#include "config.h"

// Hardware configuration
const float VEL_TO_SERVO_UNIT = 13037; // Conversion factor from rad/s to servo speed units
const float COUNTS_PER_REV = 4096;     // Encoder counts per revolution

// Servo ID configuration
const uint8_t BASE_SERVO_IDS[BASE_SERVO_COUNT] = {1, 2, 3};  // back, left, right motors
const uint8_t ARM_SERVO_IDS[ARM_SERVO_COUNT] = {4, 5, 6, 7, 8, 9, 10};  // joints 1-6 + camera_tilt

// Global servo objects
SMS_STS base_servo;
SMS_STS arm_servo;

bool initServos(uint8_t rx_pin, uint8_t tx_pin) {
    Serial2.begin(1000000, SERIAL_8N1, rx_pin, tx_pin);
    base_servo.pSerial = &Serial2; 
    arm_servo.pSerial = &Serial2;   
    return true;
}

bool checkServos(void* display) {
    Serial.println("Checking base servos...");
    delay(500);

    // Check base servos (wheel motors)
    for (int i = 0; i < BASE_SERVO_COUNT; i++) {
        uint8_t servo_id = BASE_SERVO_IDS[i];
        int pingStatus = base_servo.Ping(servo_id);
        if (pingStatus == -1) {
            Serial.printf("Base servo %d FAIL!\n", servo_id);
            delay(1000);
        } else {
            base_servo.WheelMode(servo_id);  // Set to continuous rotation mode
            base_servo.EnableTorque(servo_id, 1);
            Serial.printf("Base servo %d OK\n", servo_id);
            delay(500);
        }
    }

    Serial.println("Checking arm servos...");
    delay(500);

    // Check arm servos (position control)
    for (int i = 0; i < ARM_SERVO_COUNT; i++) {
        uint8_t servo_id = ARM_SERVO_IDS[i];
        int pingStatus = arm_servo.Ping(servo_id);
        if (pingStatus == -1) {
            Serial.printf("Arm servo %d FAIL!\n", servo_id);
            delay(1000);
        } else {
            // Servos should be in position mode by default
            arm_servo.EnableTorque(servo_id, 1);
            Serial.printf("Arm servo %d OK\n", servo_id);
            delay(500);
        }
    }

    Serial.println("All servos OK!");
    delay(500);
    return true;
}

void moveToRelaxedPositions(void* display) {
    Serial.println("Moving to relaxed positions...");
    
    // Move arm servos to their calibrated relaxed positions
    for (int i = 0; i < ARM_CALIBRATION_COUNT; i++) {
        const ServoCalibration& cal = ARM_CALIBRATION[i];
        arm_servo.WritePosEx(cal.servo_id, cal.relaxed_position, 500, 0); // 500ms time to reach position
        delay(100); // Small delay between commands
        
        Serial.printf("Servo %d -> %d\n", cal.servo_id, cal.relaxed_position);
        delay(300);
    }
    
    Serial.println("Relaxed position set!");
    delay(1000);
}


void controlBaseServo(uint8_t servo_index, float rad_per_sec, bool servos_enabled) {
    if (servo_index >= BASE_SERVO_COUNT) return;
    
    uint8_t servo_id = BASE_SERVO_IDS[servo_index];
    
    #if ENABLE_DEBUG_PRINTS
    Serial.printf("Servo %d (ID=%d): %.4f rad/s", servo_index, servo_id, rad_per_sec);
    #endif
    
    #if ENABLE_SERVO_CONTROL
    if (servos_enabled) {
        if (abs(rad_per_sec) < 0.01) {  // Dead zone - apply brake
            base_servo.WriteSpe(servo_id, 0, 255);  // Max acceleration for braking
            #if ENABLE_DEBUG_PRINTS
            Serial.printf(" -> BRAKE\n");
            #endif
        } else {
            // Normal speed command
            int16_t speed = constrain(rad_per_sec * VEL_TO_SERVO_UNIT, -4096, 4096);
            base_servo.WriteSpe(servo_id, speed, 0);  // 0 = default acceleration
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
    
    #if ENABLE_DEBUG_PRINTS
    Serial.printf("Servo %d (ID=%d): %.4f rad", servo_index, servo_id, radians);
    #endif
    
    #if ENABLE_SERVO_CONTROL
    if (servos_enabled) {
        // Use calibrated conversion function
        int16_t position = radiansToServoPosition(servo_id, radians);
        arm_servo.WritePosEx(servo_id, position, 0, 0);
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

void emergencyStopBase(void* display) {
    for (int i = 0; i < BASE_SERVO_COUNT; i++) {
        base_servo.WriteSpe(BASE_SERVO_IDS[i], 0, 255);  // Full brake
    }
    Serial.println("Base emergency stop activated!");
}

void emergencyStopArm(void* display) {
    for (int i = 0; i < ARM_SERVO_COUNT; i++) {
        arm_servo.EnableTorque(ARM_SERVO_IDS[i], 0);  // Disable torque
    }
    Serial.println("Arm emergency stop activated!");
}