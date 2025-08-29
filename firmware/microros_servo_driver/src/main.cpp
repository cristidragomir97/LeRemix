#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "config.h"
#include "utils.h"
#include "microros.h"
#include "motor_controller.h"
#include "base_subscriber.h"
#include "arm_subscriber.h"
#include "joint_publisher.h"

// Display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// System state
bool servos_enabled = false;  // Global servo enable/disable flag - DEFAULT DISABLED

// Button state tracking
unsigned long button_press_start = 0;
bool button_was_pressed = false;
bool button_long_press_detected = false;

void setup() {
  // Initialize serial for debug output first
  Serial.begin(115200);
  
  // Initialize screen
  initScreen(&display, S_SDA, S_SCL);
  displayMessage("Board started", &display);
  delay(1000);

  // Initialize button pin
  pinMode(EN_BUTTON_PIN, INPUT_PULLUP);
  displayMessage("Button initialized (long press to toggle servos)", &display);

  // Initialize servo communication
  displayMessage("Starting servo initialization...", &display);
  initServos(S_RXD, S_TXD);
  
  displayMessage("Starting servo check...", &display);
  displayMessage("Checking servos...", &display);
  checkServos(&display);
  
  // Move arm to relaxed positions after servo check
  #if ENABLE_SERVO_CONTROL
  moveToRelaxedPositions(&display);
  #endif

  // Initialize micro-ROS
  displayMessage("Initializing micro-ROS...", &display);
  displayMessage("Initializing micro-ROS...", &display);
  set_microros_serial_transports(Serial);
  
  bool ros_initialized = initMicroROS(&display);

  while (!ros_initialized) {
    displayMessage("micro-ROS init failed, retrying...", &display);
    delay(2000);
    ros_initialized = initMicroROS(&display);
  }

  displayMessage("micro-ROS initialized!", &display);
  displayMessage("micro-ROS basic initialization complete", &display);

  // Initialize subscribers and publishers
  displayMessage("Setting up subscribers and publishers...", &display);
  
  // Set up base subscriber
  setBaseServosEnabledPtr(&servos_enabled);
  setBaseDisplayPtr(&display);
  if (!initBaseSubscriber(&node, &executor, &display)) {
    displayMessage("Base subscriber failed!", &display);
    while(1) delay(1000);
  }

  // Set up arm subscriber  
  setArmServosEnabledPtr(&servos_enabled);
  setArmDisplayPtr(&display);
  if (!initArmSubscriber(&node, &executor, &display)) {
    displayMessage("Arm subscriber failed!", &display);
    while(1) delay(1000);
  }

  // Set up joint state publisher
  if (!initJointPublisher(&node, &display)) {
    displayMessage("Joint publisher failed!", &display);
    while(1) delay(1000);
  }

  displayMessage("Waiting for commands...", &display);
  displayMessage("Setup complete - waiting for ROS commands", &display);
  delay(1000);
}

void loop() {
  // Check button for servo enable/disable
  checkButtonPress(EN_BUTTON_PIN, &servos_enabled, 
                  &button_press_start, &button_was_pressed, 
                  &button_long_press_detected, &display);

  // Handle servo enable/disable toggle
  static bool last_servos_enabled = servos_enabled;
  if (servos_enabled != last_servos_enabled) {
    toggleServoControl(servos_enabled, &display);
    if (servos_enabled) {
      displayMessage("SERVOS ENABLED", &display);
    } else {
      displayMessage("SERVOS DISABLED", &display);
    }
    last_servos_enabled = servos_enabled;
  }
  
  // Run micro-ROS executor to handle incoming commands
  spinMicroRos();
  
  // Publish joint states at 200Hz (every 5ms)
  if (shouldPublishJointStates()) {
    publishJointStates();
  }
  
  // Small delay to prevent overwhelming the system
}