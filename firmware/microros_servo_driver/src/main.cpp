#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
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
SemaphoreHandle_t servos_enabled_mutex;

// Button state tracking
unsigned long button_press_start = 0;
bool button_was_pressed = false;
bool button_long_press_detected = false;

// Task handles
TaskHandle_t microros_task_handle;
TaskHandle_t servo_control_task_handle;
TaskHandle_t motor_update_task_handle;

// Inter-task communication queues
QueueHandle_t base_command_queue;
QueueHandle_t arm_command_queue;
QueueHandle_t joint_state_queue;

// Command message structures
struct BaseCommandMsg {
  float linear_x;
  float angular_z;
};

struct ArmCommandMsg {
  float joint_positions[7];  // 6 arm joints + 1 camera
  int joint_count;
};

struct JointStateMsg {
  float positions[13];  // All joint positions
  int joint_count;
};

// FreeRTOS task functions
void microros_task(void* parameter);
void servo_control_task(void* parameter);
void motor_update_task(void* parameter);

void setup() {
  // Initialize serial for debug output first
  Serial.begin(115200);
  
  // Initialize screen
  initScreen(&display, S_SDA, S_SCL);
  displayMessage("Board started", &display);
  delay(1000);
  
  // Initialize motor display system
  initMotorDisplay(&display);

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

  // Create synchronization primitives
  servos_enabled_mutex = xSemaphoreCreateMutex();
  
  // Create inter-task communication queues
  base_command_queue = xQueueCreate(5, sizeof(BaseCommandMsg));
  arm_command_queue = xQueueCreate(5, sizeof(ArmCommandMsg));
  joint_state_queue = xQueueCreate(5, sizeof(JointStateMsg));
  
  if (!servos_enabled_mutex || !base_command_queue || !arm_command_queue || !joint_state_queue) {
    displayMessage("Failed to create synchronization primitives!", &display);
    while(1) delay(1000);
  }
  
  displayMessage("Creating FreeRTOS tasks...", &display);
  
  // Create micro-ROS task on core 0 (protocol core)
  xTaskCreatePinnedToCore(
    microros_task,
    "MicroROS Task",
    8192,  // Stack size
    NULL,
    2,     // Priority
    &microros_task_handle,
    0      // Core 0
  );
  
  // Create servo control task on core 1 (control core)
  xTaskCreatePinnedToCore(
    servo_control_task,
    "Servo Control Task",
    4096,  // Stack size
    NULL,
    3,     // Higher priority for real-time control
    &servo_control_task_handle,
    1      // Core 1
  );
  
  // Create motor update task on core 1 (control core)
  xTaskCreatePinnedToCore(
    motor_update_task,
    "Motor Update Task",
    2048,  // Stack size
    NULL,
    1,     // Lower priority
    &motor_update_task_handle,
    1      // Core 1
  );
  
  displayMessage("FreeRTOS tasks created!", &display);
  displayMessage("Multi-threaded system ready", &display);
  delay(1000);
}

void loop() {
  // Empty loop - all work is now done by FreeRTOS tasks
  // This prevents the Arduino loop from interfering with task scheduling
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// Micro-ROS task running on Core 0
void microros_task(void* parameter) {
  // Initialize micro-ROS
  displayMessage("Initializing micro-ROS...", &display);
  set_microros_serial_transports(Serial);
  
  bool ros_initialized = initMicroROS(&display);

  while (!ros_initialized) {
    displayMessage("micro-ROS init failed, retrying...", &display);
    vTaskDelay(pdMS_TO_TICKS(2000));
    ros_initialized = initMicroROS(&display);
  }

  displayMessage("micro-ROS initialized!", &display);

  // Initialize subscribers and publishers
  displayMessage("Setting up subscribers and publishers...", &display);
  
  // Set up base subscriber
  setBaseServosEnabledPtr(&servos_enabled);
  setBaseDisplayPtr(&display);
  setBaseCommandQueue(base_command_queue);
  if (!initBaseSubscriber(&node, &executor, &display)) {
    displayMessage("Base subscriber failed!", &display);
    vTaskDelete(NULL);
  }

  // Set up arm subscriber  
  setArmServosEnabledPtr(&servos_enabled);
  setArmDisplayPtr(&display);
  setArmCommandQueue(arm_command_queue);
  if (!initArmSubscriber(&node, &executor, &display)) {
    displayMessage("Arm subscriber failed!", &display);
    vTaskDelete(NULL);
  }

  // Set up joint state publisher
  if (!initJointPublisher(&node, &display)) {
    displayMessage("Joint publisher failed!", &display);
    vTaskDelete(NULL);
  }

  displayMessage("micro-ROS ready!", &display);
  
  // Main micro-ROS loop
  for (;;) {
    // Run micro-ROS executor to handle incoming commands
    spinMicroRos();
    
    // Publish joint states at 200Hz (every 5ms)
    if (shouldPublishJointStates()) {
      publishJointStates();
    }
    
    // Small delay to allow other tasks to run
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// Servo control task running on Core 1
void servo_control_task(void* parameter) {
  displayMessage("Servo control task started", &display);
  
  BaseCommandMsg base_cmd;
  ArmCommandMsg arm_cmd;
  
  for (;;) {
    // Check button for servo enable/disable (with mutex protection)
    bool current_enabled = false;
    if (xSemaphoreTake(servos_enabled_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      current_enabled = servos_enabled;
      xSemaphoreGive(servos_enabled_mutex);
    }
    
    checkButtonPress(EN_BUTTON_PIN, &servos_enabled, 
                    &button_press_start, &button_was_pressed, 
                    &button_long_press_detected, &display);

    // Handle servo enable/disable toggle (with mutex protection)
    static bool last_servos_enabled = false;
    bool new_enabled = false;
    if (xSemaphoreTake(servos_enabled_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      new_enabled = servos_enabled;
      xSemaphoreGive(servos_enabled_mutex);
    }
    
    if (new_enabled != last_servos_enabled) {
      toggleServoControl(new_enabled, &display);
      if (new_enabled) {
        displayMessage("SERVOS ENABLED", &display);
      } else {
        displayMessage("SERVOS DISABLED", &display);
      }
      last_servos_enabled = new_enabled;
    }
    
    // Process base commands from queue
    if (xQueueReceive(base_command_queue, &base_cmd, 0) == pdTRUE) {
      // Get servo enabled state safely
      bool servos_active = false;
      if (xSemaphoreTake(servos_enabled_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        servos_active = servos_enabled;
        xSemaphoreGive(servos_enabled_mutex);
      }
      
      if (servos_active) {
        // Convert linear and angular velocity to wheel speeds
        // This is robot-specific - adjust based on your base configuration
        float left_speed = base_cmd.linear_x - base_cmd.angular_z;
        float right_speed = base_cmd.linear_x + base_cmd.angular_z;
        
        // Control base servos (assuming differential drive)
        controlBaseServo(0, left_speed, true);   // Left wheel
        controlBaseServo(1, right_speed, true);  // Right wheel
        if (BASE_SERVO_COUNT > 2) {
          controlBaseServo(2, 0.0, true);        // Additional servo if present
        }
      }
    }
    
    // Process arm commands from queue
    if (xQueueReceive(arm_command_queue, &arm_cmd, 0) == pdTRUE) {
      // Get servo enabled state safely
      bool servos_active = false;
      if (xSemaphoreTake(servos_enabled_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        servos_active = servos_enabled;
        xSemaphoreGive(servos_enabled_mutex);
      }
      
      if (servos_active) {
        // Control arm servos with received joint positions
        for (int i = 0; i < arm_cmd.joint_count && i < ARM_SERVO_COUNT; i++) {
          controlArmServo(i, arm_cmd.joint_positions[i], true);
        }
      }
    }
    
    // Task runs at ~100Hz
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Motor update task running on Core 1
void motor_update_task(void* parameter) {
  displayMessage("Motor update task started", &display);
  
  for (;;) {
    // Update all motor states
    updateAllMotors();
    
    // Collect joint states and send to queue if needed
    JointStateMsg joint_msg;
    // TODO: Populate joint_msg with current joint states
    // xQueueSend(joint_state_queue, &joint_msg, 0);
    
    // Run at ~50Hz
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}