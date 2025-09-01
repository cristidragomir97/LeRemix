#include "motor_controller.h"
#include "utils.h"
#include "config.h"

// Servo ID configuration
const uint8_t BASE_SERVO_IDS[BASE_SERVO_COUNT] = {2, 3, 1};  // back, left, right motors
const uint8_t ARM_SERVO_IDS[ARM_SERVO_COUNT] = {4, 5, 6, 7, 8, 9, 10};  // joints 1-6 + camera_tilt

// Global servo objects (using SMS_STS library)
SMS_STS base_servo;
SMS_STS arm_servo;

// Global motor objects (using Motor wrapper)
Motor baseMotors[BASE_SERVO_COUNT] = {
  Motor(BASE_SERVO_IDS[0], &base_servo, true),  // back - isBase=true
  Motor(BASE_SERVO_IDS[1], &base_servo, true),  // left - isBase=true  
  Motor(BASE_SERVO_IDS[2], &base_servo, true)   // right - isBase=true
};
Motor armMotors[ARM_SERVO_COUNT] = {
  Motor(ARM_SERVO_IDS[0], &arm_servo, false),   // joint 1 - isBase=false
  Motor(ARM_SERVO_IDS[1], &arm_servo, false),   // joint 2 - isBase=false
  Motor(ARM_SERVO_IDS[2], &arm_servo, false),   // joint 3 - isBase=false
  Motor(ARM_SERVO_IDS[3], &arm_servo, false),   // joint 4 - isBase=false
  Motor(ARM_SERVO_IDS[4], &arm_servo, false),   // joint 5 - isBase=false
  Motor(ARM_SERVO_IDS[5], &arm_servo, false),   // joint 6 - isBase=false
  Motor(ARM_SERVO_IDS[6], &arm_servo, false)    // camera_tilt - isBase=false
};

bool initServos(uint8_t rx_pin, uint8_t tx_pin) {
    Serial2.begin(1000000, SERIAL_8N1, rx_pin, tx_pin);
    
    // Initialize servo objects
    base_servo.pSerial = &Serial2; 
    arm_servo.pSerial = &Serial2;   
    
    // Initialize all motor objects
    for (int i = 0; i < BASE_SERVO_COUNT; i++) {
        if (!baseMotors[i].init()) {
            return false;
        }
    }
    
    for (int i = 0; i < ARM_SERVO_COUNT; i++) {
        if (!armMotors[i].init()) {
            return false;
        }
    }
    
    return true;
}

void initMotorDisplay(Adafruit_SSD1306* display) {
    Motor::setDisplay(display);
}

bool checkServos(Adafruit_SSD1306* display) {
    displayMessage("Checking base servos...", display);
    delay(500);

    // Check base servos (wheel motors)
    for (int i = 0; i < BASE_SERVO_COUNT; i++) {
        uint8_t servo_id = BASE_SERVO_IDS[i];
        bool pingResult = baseMotors[i].ping();
        if (!pingResult) {
            char buf[64];
            sprintf(buf, "Base servo %d FAIL!", servo_id);
            displayMessage(buf, display);
            delay(1000);
        } else {
            char buf[64];
            // Set to wheel mode for base servos
            base_servo.WheelMode(servo_id);
            base_servo.EnableTorque(servo_id, 1);
            sprintf(buf, "Base servo %d OK", servo_id);
            displayMessage(buf, display);
            delay(500);
        }
    }

    displayMessage("Checking arm servos...", display);
    delay(500);

    // Check arm servos (position control)
    for (int i = 0; i < ARM_SERVO_COUNT; i++) {
        uint8_t servo_id = ARM_SERVO_IDS[i];
        bool pingResult = armMotors[i].ping();
        if (!pingResult) {
            char buf[64];
            sprintf(buf, "Arm servo %d FAIL!", servo_id);
            displayMessage(buf, display);
            delay(1000);
        } else {
            char buf[64];
            // Arm servos stay in position mode (default)
            arm_servo.EnableTorque(servo_id, 1);
            sprintf(buf, "Arm servo %d OK", servo_id);
            displayMessage(buf, display);
            delay(500);
        }
    }

    displayMessage("All servos OK!", display);
    delay(500);
    return true;
}

void moveToRelaxedPositions(Adafruit_SSD1306* display) {
    displayMessage("Moving to relaxed positions...", display);
    
    // Move arm servos to their calibrated relaxed positions
    for (int i = 0; i < ARM_CALIBRATION_COUNT; i++) {
        const ServoCalibration& cal = ARM_CALIBRATION[i];
        
        // Find the motor with matching servo ID
        Motor* motor = nullptr;
        for (int j = 0; j < ARM_SERVO_COUNT; j++) {
            if (armMotors[j].getId() == cal.servo_id) {
                motor = &armMotors[j];
                break;
            }
        }
        
        if (motor != nullptr) {
            motor->setPosition(cal.relaxed_position);
            delay(100); // Small delay between commands
            
            char buf[64];
            sprintf(buf, "Servo %d -> %d", cal.servo_id, cal.relaxed_position);
            displayMessage(buf, display);
            delay(300);
        }
    }
    
    displayMessage("Relaxed position set!", display);
    delay(1000);
}

void toggleServoControl(bool enable, Adafruit_SSD1306* display) {
    if (!enable) {
        // Disable all servos
        #if ENABLE_SERVO_CONTROL
        for (int i = 0; i < BASE_SERVO_COUNT; i++) {
            baseMotors[i].setSpeed(0.0f);  // Stop base servos with braking
        }
        for (int i = 0; i < ARM_SERVO_COUNT; i++) {
            armMotors[i].setTorqueEnable(false);  // Disable arm servo torque
        }
        #endif
        displayMessage("SERVOS DISABLED", display);
    } else {
        // Re-enable all servos
        #if ENABLE_SERVO_CONTROL
        for (int i = 0; i < BASE_SERVO_COUNT; i++) {
            baseMotors[i].setTorqueEnable(true);  // Re-enable base servo torque
        }
        for (int i = 0; i < ARM_SERVO_COUNT; i++) {
            armMotors[i].setTorqueEnable(true);  // Re-enable arm servo torque
        }
        #endif
        displayMessage("SERVOS ENABLED", display);
    }
}

void controlBaseServo(uint8_t servo_index, float rad_per_sec, bool servos_enabled) {
    if (servo_index >= BASE_SERVO_COUNT) return;
    
    Motor& motor = baseMotors[servo_index];
    uint8_t servo_id = motor.getId();
    
    #if ENABLE_SERVO_CONTROL
    if (servos_enabled) {
        bool result = motor.setSpeed(rad_per_sec);
        // Motor class will handle display updates
    }
    #endif
}

void controlArmServo(uint8_t servo_index, float radians, bool servos_enabled) {
    if (servo_index >= ARM_SERVO_COUNT) return;
    
    Motor& motor = armMotors[servo_index];
    uint8_t servo_id = motor.getId();
    
    #if ENABLE_SERVO_CONTROL
    if (servos_enabled) {
        // Use calibrated conversion function
        int16_t position = radiansToServoPosition(servo_id, radians);
        bool result = motor.setPosition(position);
        // Motor class will handle display updates
    }
    #endif
}

void emergencyStopBase(Adafruit_SSD1306* display) {
    for (int i = 0; i < BASE_SERVO_COUNT; i++) {
        baseMotors[i].setSpeed(0.0f);  // Brake using position mode
    }
    displayMessage("Base emergency stop activated!", display);
}

void emergencyStopArm(Adafruit_SSD1306* display) {
    for (int i = 0; i < ARM_SERVO_COUNT; i++) {
        armMotors[i].setTorqueEnable(false);  // Disable torque
    }
    displayMessage("Arm emergency stop activated!", display);
}

void updateAllMotors() {
    // Update all motor states - call this regularly in main loop
    for (int i = 0; i < BASE_SERVO_COUNT; i++) {
        baseMotors[i].update();
    }
    
    for (int i = 0; i < ARM_SERVO_COUNT; i++) {
        armMotors[i].update();
    }
}