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
  Serial1.begin(115200, SERIAL_8N1, 15, 2); // Servo serial port
  Serial1.println("Simple MicroROS Servo Driver Starting...");
  
  // Initialize servo communication
  Serial1.println("Initializing servos...");
  initServos(S_RXD, S_TXD);
  
  Serial1.println("Checking servos...");
  checkServos(nullptr);
  
  // Move arm to relaxed positions after servo check
  #if ENABLE_SERVO_CONTROL
  moveToRelaxedPositions(nullptr);
  #endif

  // Initialize micro-ROS
  Serial1.println("Initializing micro-ROS...");
  set_microros_serial_transports(Serial);
  
  bool ros_initialized = initMicroROS(nullptr);

  while (!ros_initialized) {
    Serial1.println("micro-ROS init failed, retrying...");
    delay(2000);
    ros_initialized = initMicroROS(nullptr);
  }

  Serial1.println("micro-ROS initialized!");

  // Initialize subscribers and publishers
  Serial1.println("Setting up subscribers and publishers...");
  
  // Set up base subscriber
  setBaseServosEnabledPtr(&servos_enabled);
  setBaseDisplayPtr(nullptr);
  if (!initBaseSubscriber(&node, &executor, nullptr)) {
    Serial1.println("Base subscriber failed!");
    while(1) delay(1000);
  }

  // Set up arm subscriber  
  setArmServosEnabledPtr(&servos_enabled);
  setArmDisplayPtr(nullptr);
  if (!initArmSubscriber(&node, &executor, nullptr)) {
    Serial1.println("Arm subscriber failed!");
    while(1) delay(1000);
  }

  // Set up joint state publisher
 // if (!initJointPublisher(&node, nullptr)) {
  //  Serial1.println("Joint publisher failed!");
  //  while(1) delay(1000);
 // }

  Serial1.println("Setup complete - waiting for ROS commands");
}

void loop() {
  // Run micro-ROS executor to handle incoming commands
  spinMicroRos();
  
  // Publish joint states at 200Hz (every 5ms)
  //if (shouldPublishJointStates()) {
  //  publishJointStates();
  //}
}