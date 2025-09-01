#include <Arduino.h>
#include "STSServoDriver.h"

// Servo driver object
STSServoDriver servos;

// Conversion constants
const float WHEEL_RADIUS = 0.05;  // 50mm = 0.05m
const float SERVO_COUNTS_PER_REV = 4096;  // Servo has 4096 counts per revolution


// Convert rad/s to servo speed units
float radPerSecToServoSpeed(float rad_per_sec) {
  // Convert rad/s to rev/s, then to servo speed units
  float rev_per_sec = rad_per_sec / (2 * PI);
  return rev_per_sec * SERVO_COUNTS_PER_REV;
}

// Motor class that wraps each servo
class Motor {
private:
  uint8_t id;
  int lastPosition;
  float currentSpeed;
  bool isMoving;
  unsigned long lastUpdateTime;
  
public:
  Motor(uint8_t servoId) : id(servoId), lastPosition(0), 
                          currentSpeed(0.0), isMoving(false), 
                          lastUpdateTime(0) {}
  
  void init() {
    lastPosition = servos.getCurrentPosition(id);
    lastUpdateTime = millis();
  }
  
  void setSpeed(float rad_per_sec) {
    if (rad_per_sec == 0.0) {
      // Request to brake - immediately lock at current position
      if (isMoving) {
        Serial.printf("Motor %d: BRAKING - locking at current position\n", id);
        
        // Read current position and immediately lock there
        int currentPos = servos.getCurrentPosition(id);
        if (currentPos != -1) {
          // Switch to position mode and lock at current position
          servos.setMode(id, STSMode::POSITION);
          delayMicroseconds(100); // 50ms delay for mode switch
          servos.setTargetPosition(id, currentPos);
          
          Serial.printf("Motor %d: STOPPED and locked at position %d\n", id, currentPos);
        } else {
          Serial.printf("Motor %d: Error reading position for braking\n", id);
        }
        
        // Reset state
        isMoving = false;
        currentSpeed = 0.0;
      }
    } else {
      // Request to move
      Serial.printf("Motor %d: Setting speed to %.3f rad/s\n", id, rad_per_sec);
      
      currentSpeed = rad_per_sec;
      isMoving = true;
      
      // Set velocity mode and start moving
      servos.setMode(id, STSMode::VELOCITY);
      delay(50);
      int speed_int = (int)(radPerSecToServoSpeed(rad_per_sec));
      servos.setTargetVelocity(id, speed_int);
      
      Serial.printf("Motor %d: Moving at servo velocity %d\n", id, speed_int);
    }
  }
  
  void update() {
    unsigned long currentTime = millis();
    
    // Update position tracking
    int currentPos = servos.getCurrentPosition(id);
    if (currentPos != -1) {
      lastPosition = currentPos;
    }
    
    // Simple update - no complex state machine needed anymore
    // All braking logic is handled immediately in setSpeed()
    
    lastUpdateTime = currentTime;
  }
  
  // Getter functions
  bool getIsMoving() const { return isMoving; }
  float getCurrentSpeed() const { return currentSpeed; }
  int getLastPosition() const { return lastPosition; }
  uint8_t getId() const { return id; }
};

// Motor instances
Motor baseMotors[3] = {Motor(1), Motor(2), Motor(3)};
Motor armMotors[7] = {Motor(4), Motor(5), Motor(6), Motor(7), Motor(8), Motor(9), Motor(11)};

// Servo IDs (kept for compatibility)
const uint8_t BASE_SERVO_IDS[] = {1, 2, 3};  // Continuous rotation servos
const uint8_t ARM_SERVO_IDS[] = {4, 5, 6, 7, 8, 9, 11};  // Position servos (including 11th)



// Helper function to get motor by ID
Motor* getMotorById(uint8_t servo_id) {
  for (int i = 0; i < 3; i++) {
    if (baseMotors[i].getId() == servo_id) {
      return &baseMotors[i];
    }
  }
  for (int i = 0; i < 7; i++) {
    if (armMotors[i].getId() == servo_id) {
      return &armMotors[i];
    }
  }
  return nullptr;
}

// Set speed for a servo using Motor object
void setSpeed(uint8_t servo_id, float rad_per_sec) {
  Motor* motor = getMotorById(servo_id);
  if (motor != nullptr) {
    motor->setSpeed(rad_per_sec);
  } else {
    Serial.printf("Error: Motor with ID %d not found\n", servo_id);
  }
}



void initializeServos() {
  Serial.println("Initializing servos...");
  
  // Initialize servo driver
  Serial2.begin(1000000, SERIAL_8N1, 18, 19);
  if (servos.init(&Serial2)) {
    Serial.println("Servo driver initialized successfully!");
  } else {
    Serial.println("Failed to initialize servo driver!");
    return;
  }
  
  // Initialize all motor objects
  for (int i = 0; i < 3; i++) {
    baseMotors[i].init();
  }
  
  for (int i = 0; i < 7; i++) {
    armMotors[i].init();
  }
  
  Serial.println("Servo initialization complete!");
}

void testBaseServo(int servoNum) {
  uint8_t id = BASE_SERVO_IDS[servoNum - 1];
  Serial.printf("Starting base servo %d (ID=%d) - continuous rotation\n", servoNum, id);
  
  // Start rotating at 2 rad/s clockwise
  setSpeed(id, 12.0);  // 2 rad/s clockwise
  
  Serial.printf("Base servo %d now rotating continuously at 2 rad/s - send 's' to brake\n", servoNum);
}

void testArmServo(int servoNum) {
  uint8_t id = servoNum;  // Servo ID matches the number
  Serial.printf("Testing arm servo %d (ID=%d) - moving to test positions\n", servoNum, id);
  
  // Set to position mode
  servos.setMode(id, STSMode::POSITION);
  delay(100);
  
  // Get current position
  int currentPos = servos.getCurrentPosition(id);
  if (currentPos == -1) {
    Serial.printf("Error: Could not read position from servo %d\n", servoNum);
    return;
  }
  
  Serial.printf("Current position: %d\n", currentPos);
  
  // Move to position +500 from current
  int targetPos1 = currentPos + 500;
  if (targetPos1 > 4095) targetPos1 = 4095;
  
  Serial.printf("Moving to position %d...\n", targetPos1);
  servos.setTargetPosition(id, targetPos1);
  delay(100);
  while (servos.isMoving(id)) {
    delay(50);
  }
  
  // Move to position -500 from original
  int targetPos2 = currentPos - 500;
  if (targetPos2 < 0) targetPos2 = 0;
  
  Serial.printf("Moving to position %d...\n", targetPos2);
  servos.setTargetPosition(id, targetPos2);
  delay(100);
  while (servos.isMoving(id)) {
    delay(50);
  }
  
  // Return to original position
  Serial.printf("Returning to original position %d...\n", currentPos);
  servos.setTargetPosition(id, currentPos);
  delay(100);
  while (servos.isMoving(id)) {
    delay(50);
  }
  
  Serial.printf("Arm servo %d test complete\n", servoNum);
}

void stopAllServos() {
  Serial.println("Stopping all base servos with position braking...");
  
  // Stop base servos using setSpeed(0) for position braking
  for (int i = 0; i < 3; i++) {
    setSpeed(BASE_SERVO_IDS[i], 0.0);  // Brake using position mode
  }
  
  Serial.println("All base servos stopped and locked");
}

void pingAllServos() {
  Serial.println("Pinging all servos...");
  
  // Ping base servos
  Serial.println("Base servos (continuous rotation):");
  for (int i = 0; i < 3; i++) {
    uint8_t id = BASE_SERVO_IDS[i];
    bool result = servos.ping(id);
    if (result) {
      Serial.printf("  Servo %d (ID=%d): ONLINE\n", i+1, id);
    } else {
      Serial.printf("  Servo %d (ID=%d): OFFLINE\n", i+1, id);
    }
    delay(100);
  }
  
  // Ping arm servos
  Serial.println("Arm servos (position control):");
  for (int i = 0; i < 7; i++) {
    uint8_t id = ARM_SERVO_IDS[i];
    bool result = servos.ping(id);
    if (result) {
      Serial.printf("  Servo %d (ID=%d): ONLINE\n", id, id);
    } else {
      Serial.printf("  Servo %d (ID=%d): OFFLINE\n", id, id);
    }
    delay(100);
  }
}

void setup() {
  Serial.begin(115200);
  
  Serial.println("=== SERVO TEST SKETCH ===");
  Serial.println("Commands:");
  Serial.println("  b1, b2, b3 - Test base servos (continuous rotation)");
  Serial.println("  a4, a5, a6, a7, a8, a9, a11 - Test arm servos (position)");
  Serial.println("  stop - Stop all servos");
  Serial.println("  ping - Ping all servos");
  Serial.println("=========================");
  
  // Initialize all servos
  initializeServos();
}

void loop() {
  // Update all motor objects continuously
  for (int i = 0; i < 3; i++) {
    baseMotors[i].update();
  }
  
  for (int i = 0; i < 7; i++) {
    armMotors[i].update();
  }
  
  // Handle serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command == "stop") {
      stopAllServos();
    } else if (command == "ping") {
      pingAllServos();
    } else if (command.startsWith("b")) {
      // Base servo command
      int servoNum = command.substring(1).toInt();
      if (servoNum >= 1 && servoNum <= 3) {
        testBaseServo(servoNum);
      } else {
        Serial.println("Invalid base servo number. Use b1, b2, or b3");
      }
    } else if (command.startsWith("a")) {
      // Arm servo command
      int servoNum = command.substring(1).toInt();
      if ((servoNum >= 4 && servoNum <= 9) || servoNum == 11) {
        testArmServo(servoNum);
      } else {
        Serial.println("Invalid arm servo number. Use a4, a5, a6, a7, a8, a9, or a11");
      }
    } else {
      Serial.println("Unknown command. Type 'ping', 'stop', 'b1-b3', or 'a4-a9,a11'");
    }
  }
}