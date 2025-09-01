#include "Motor.h"
#include "utils.h"

// Static display pointer initialization
Adafruit_SSD1306* Motor::display = nullptr;

Motor::Motor(uint8_t servoId, SMS_STS* servoObject, bool isBase) 
  : id(servoId), servo(servoObject), lastPosition(0), 
    currentSpeed(0.0f), isMoving(false), lastUpdateTime(0), isBaseMotor(isBase) {
}

void Motor::setDisplay(Adafruit_SSD1306* displayPtr) {
  display = displayPtr;
}

bool Motor::init() {
  if (servo == nullptr) {
    return false;
  }
  
  // Enable torque for the servo
  servo->EnableTorque(id, 1);
  
  if (isBaseMotor) {
    // Set base motors to wheel mode (continuous rotation)
    servo->WheelMode(id);
  } else {
    // Arm motors stay in position mode (default)
  }
  
  // Read initial position
  int32_t pos = servo->ReadPos(id);
  if (pos != -1) {
    lastPosition = pos;
  }
  
  lastUpdateTime = millis();
  return true;
}

bool Motor::setSpeed(float rad_per_sec) {
  if (servo == nullptr) {
    return false;
  }
  
  if (rad_per_sec == 0.0f) {
    // Request to brake - immediately lock at current position
    if (isMoving && isBaseMotor) {
      // Read current position and immediately lock there
      int32_t currentPos = servo->ReadPos(id);
      if (currentPos != -1) {
        // Switch to position mode and lock at current position
        servo->WritePosEx(id, currentPos, 0, 50); // Move to current position with 50ms time
        
        // Display update for braking
        if (display != nullptr) {
          char buf[64];
          sprintf(buf, "M%d: BRAKE @%d", id, currentPos);
          displayMessage(buf, display);
        }
        
        // Successfully locked position
        isMoving = false;
        currentSpeed = 0.0f;
        lastPosition = currentPos;
        return true;
      } else {
        // Fallback - just stop with zero speed
        servo->WriteSpe(id, 0, 255); // Stop with max acceleration
        isMoving = false;
        currentSpeed = 0.0f;
        return true;
      }
    } else {
      // Stop using velocity command
      if (isBaseMotor) {
        servo->WriteSpe(id, 0, 255); // Stop with max acceleration
      }
      currentSpeed = 0.0f;
      isMoving = false;
      return true;
    }
  } else {
    // Request to move
    currentSpeed = rad_per_sec;
    isMoving = true;
    
    if (isBaseMotor) {
      // Make sure we're in wheel mode
      servo->WheelMode(id);
      // Convert rad/s to servo speed units and send
      int16_t speed_servo_units = radPerSecToServoSpeed(rad_per_sec);
      servo->WriteSpe(id, speed_servo_units, 0); // 0 = default acceleration
      
      // Display update
      if (display != nullptr) {
        char buf[64];
        sprintf(buf, "M%d: %.2f rad/s", id, rad_per_sec);
        displayMessage(buf, display);
      }
    }
    
    return true;
  }
}

bool Motor::setPosition(int16_t position) {
  if (servo == nullptr) {
    return false;
  }
  
  // Set target position (this automatically sets position mode)
  servo->WritePosEx(id, position, 500, 0); // 500ms time to reach position
  
  // Display update for position command
  if (display != nullptr) {
    char buf[64];
    sprintf(buf, "M%d: POS %d", id, position);
    displayMessage(buf, display);
  }
  
  // Update internal state
  isMoving = true;
  currentSpeed = 0.0f; // Position mode doesn't use speed
  
  return true;
}

bool Motor::setTorqueEnable(bool enable) {
  if (servo == nullptr) {
    return false;
  }
  
  servo->EnableTorque(id, enable ? 1 : 0);
  return true;
}

int16_t Motor::getCurrentPosition() {
  if (servo == nullptr) {
    return -1;
  }
  
  int32_t pos = servo->ReadPos(id);
  return (pos == -1) ? -1 : (int16_t)pos;
}

int16_t Motor::getCurrentVelocity() {
  if (servo == nullptr) {
    return -1;
  }
  
  int16_t speed = servo->ReadSpeed(id);
  return speed;
}

bool Motor::ping() {
  if (servo == nullptr) {
    return false;
  }
  
  return (servo->Ping(id) != -1);
}

void Motor::update() {
  if (servo == nullptr) {
    return;
  }
  
  unsigned long currentTime = millis();
  
  // Update position tracking
  int32_t currentPos = servo->ReadPos(id);
  if (currentPos != -1) {
    lastPosition = currentPos;
  }
  
  // Check if we're still moving (for position mode operations)
  if (isMoving && currentSpeed == 0.0f && !isBaseMotor) {
    // We're in position mode (arm motor), check if movement is complete
    // For simplicity, assume movement completes after 1 second
    if (currentTime - lastUpdateTime > 1000) {
      isMoving = false;
    }
  }
  
  lastUpdateTime = currentTime;
}

int16_t Motor::radPerSecToServoSpeed(float rad_per_sec) {
  // Use the proven conversion from the working system
  const float VEL_TO_SERVO_UNIT = 13037.0f; // Conversion factor from rad/s to servo speed units
  
  int16_t speed = constrain(rad_per_sec * VEL_TO_SERVO_UNIT, -4096, 4096);
  
  return speed;
}

float Motor::servoSpeedToRadPerSec(int16_t servo_speed) {
  // Convert servo speed units to rev/s, then to rad/s
  float rev_per_sec = (float)servo_speed / SERVO_COUNTS_PER_REV;
  return rev_per_sec * RAD_PER_REV;
}