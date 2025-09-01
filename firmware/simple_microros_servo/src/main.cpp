#include <Arduino.h>
#include "config.h"
#include "microros.h"
#include "motor_controller.h"
#include "base_subscriber.h"
#include "arm_subscriber.h"
#include "joint_publisher.h"

// System state
bool servos_enabled = true;  // Global servo enable/disable flag - DEFAULT ENABLED

void setup() {
  // Initialize serial for debug output first
  Serial.begin(115200);
  Serial.println("Simple MicroROS Servo Driver Starting...");
  
  // Initialize servo communication
  Serial.println("Initializing servos...");
  initServos(S_RXD, S_TXD);
  
  Serial.println("Checking servos...");
  checkServos(nullptr);
  
  // Move arm to relaxed positions after servo check
  #if ENABLE_SERVO_CONTROL
  moveToRelaxedPositions(nullptr);
  #endif

  // Initialize micro-ROS
  Serial.println("Initializing micro-ROS...");
  set_microros_serial_transports(Serial);
  
  bool ros_initialized = initMicroROS(nullptr);

  while (!ros_initialized) {
    Serial.println("micro-ROS init failed, retrying...");
    delay(2000);
    ros_initialized = initMicroROS(nullptr);
  }

  Serial.println("micro-ROS initialized!");

  // Initialize subscribers and publishers
  Serial.println("Setting up subscribers and publishers...");
  
  // Set up base subscriber
  setBaseServosEnabledPtr(&servos_enabled);
  setBaseDisplayPtr(nullptr);
  if (!initBaseSubscriber(&node, &executor, nullptr)) {
    Serial.println("Base subscriber failed!");
    while(1) delay(1000);
  }

  // Set up arm subscriber  
  setArmServosEnabledPtr(&servos_enabled);
  setArmDisplayPtr(nullptr);
  if (!initArmSubscriber(&node, &executor, nullptr)) {
    Serial.println("Arm subscriber failed!");
    while(1) delay(1000);
  }

  // Set up joint state publisher
  if (!initJointPublisher(&node, nullptr)) {
    Serial.println("Joint publisher failed!");
    while(1) delay(1000);
  }

  Serial.println("Setup complete - waiting for ROS commands");
}

void loop() {
  // Run micro-ROS executor to handle incoming commands
  spinMicroRos();
  
  // Publish joint states at 200Hz (every 5ms)
  if (shouldPublishJointStates()) {
    publishJointStates();
  }
}