#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <sensor_msgs/msg/joint_state.h>
#include <SCServo.h>
#include <config.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

// Topic names matching leremix_control_plugin
#define BASE_CMD_TOPIC   "/esp32/base_cmd"
#define ARM_CMD_TOPIC    "/esp32/arm_cmd"
#define JOINT_STATE_TOPIC "/esp32/joint_states"

#define S_RXD 18
#define S_TXD 19

SMS_STS base_servo;  // For base wheels (continuous rotation)
SCSCL arm_servo;     // For arm joints (position control)


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Hardware configuration
const float VEL_TO_SERVO_UNIT = 13037; // Conversion factor from rad/s to servo speed units
const float COUNTS_PER_REV = 4096;     // Encoder counts per revolution

// Servo ID configuration
#define BASE_SERVO_COUNT 3
#define ARM_SERVO_COUNT 7  // 6 arm joints + 1 camera tilt

const uint8_t BASE_SERVO_IDS[BASE_SERVO_COUNT] = {1, 2, 3};  // back, left, right motors
const uint8_t ARM_SERVO_IDS[ARM_SERVO_COUNT] = {4, 5, 6, 7, 8, 9, 10};  // joints 1-6 + camera_tilt

// Global micro-ROS objects
rcl_node_t node;
rclc_executor_t executor;
static rclc_support_t support;
static rcl_allocator_t allocator;

// Subscribers
rcl_subscription_t base_cmd_sub;
rcl_subscription_t arm_cmd_sub;
std_msgs__msg__Float64MultiArray base_cmd_msg;
std_msgs__msg__Float64MultiArray arm_cmd_msg;

// Publishers
rcl_publisher_t joint_state_pub;
sensor_msgs__msg__JointState joint_state_msg;

// Control timing
unsigned long last_joint_state_time = 0;
const unsigned long CONTROL_PERIOD_MS = 5;  // 200Hz = 5ms period

// Watchdog mechanism
unsigned long last_base_cmd_time = 0;
unsigned long last_arm_cmd_time = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 500;  // 500ms timeout
bool watchdog_enabled = false;
bool base_emergency_stop = false;
bool arm_emergency_stop = false;

void displayMessage(const char* msg) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(msg);
  display.display();
}

void initScreen(){
  Wire.begin(S_SDA, S_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.display();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  // Row1.
}

// Base command callback - expects Float64MultiArray with 3 values (rad/s)
void base_cmd_callback(const void *msgin) {
  const std_msgs__msg__Float64MultiArray *msg = (std_msgs__msg__Float64MultiArray *)msgin;
  
  // Update watchdog timestamp
  last_base_cmd_time = millis();
  base_emergency_stop = false;  // Clear emergency stop when receiving commands
  
  if (msg->data.size >= BASE_SERVO_COUNT) {
    // Convert rad/s to servo speed units and send to base servos
    for (int i = 0; i < BASE_SERVO_COUNT; i++) {
      float rad_per_sec = msg->data.data[i];
      
      if (abs(rad_per_sec) < 0.01) {  // Dead zone - apply brake
        // Stop the motor with high acceleration (immediate stop)
        base_servo.WriteSpe(BASE_SERVO_IDS[i], 0, 255);  // Max acceleration for braking
      } else {
        // Normal speed command
        int16_t speed = constrain(rad_per_sec * VEL_TO_SERVO_UNIT, -4096, 4096);
        base_servo.WriteSpe(BASE_SERVO_IDS[i], speed, 0);  // 0 = default acceleration
      }
    }
    
    char buf[128];
    sprintf(buf, "Base: %.2f %.2f %.2f", 
            msg->data.data[0], msg->data.data[1], msg->data.data[2]);
    displayMessage(buf);
  }
}

// Arm command callback - expects Float64MultiArray with 7 values (radians)
void arm_cmd_callback(const void *msgin) {
  const std_msgs__msg__Float64MultiArray *msg = (std_msgs__msg__Float64MultiArray *)msgin;
  
  // Update watchdog timestamp
  last_arm_cmd_time = millis();
  
  // Re-enable torque if recovering from emergency stop
  if (arm_emergency_stop) {
    for (int i = 0; i < ARM_SERVO_COUNT; i++) {
      arm_servo.EnableTorque(ARM_SERVO_IDS[i], 1);  // Re-enable torque
    }
    arm_emergency_stop = false;
  }
  
  if (msg->data.size >= ARM_SERVO_COUNT) {
    // Convert radians to servo position units and send to arm servos
    for (int i = 0; i < ARM_SERVO_COUNT; i++) {
      float radians = msg->data.data[i];
      // Use calibrated conversion function
      int16_t position = radiansToServoPosition(ARM_SERVO_IDS[i], radians);
      arm_servo.WritePosEx(ARM_SERVO_IDS[i], position, 0, 0);
    }
    
    char buf[128];
    sprintf(buf, "Arm: %.2f %.2f %.2f %.2f", 
            msg->data.data[0], msg->data.data[1], msg->data.data[2], msg->data.data[3]);
    displayMessage(buf);
  }
}

void checkWatchdog() {
  if (!watchdog_enabled) return;
  
  unsigned long current_time = millis();
  
  // Check base command timeout
  if (current_time - last_base_cmd_time > WATCHDOG_TIMEOUT_MS && !base_emergency_stop) {
    base_emergency_stop = true;
    // Emergency stop all base motors
    for (int i = 0; i < BASE_SERVO_COUNT; i++) {
      base_servo.WriteSpe(BASE_SERVO_IDS[i], 0, 255);  // Full brake
    }
    displayMessage("WATCHDOG: Base stopped!");
  }
  
  // Check arm command timeout
  if (current_time - last_arm_cmd_time > WATCHDOG_TIMEOUT_MS && !arm_emergency_stop) {
    arm_emergency_stop = true;
    // For arm servos, disable torque to prevent damage
    for (int i = 0; i < ARM_SERVO_COUNT; i++) {
      arm_servo.EnableTorque(ARM_SERVO_IDS[i], 0);  // Disable torque
    }
    displayMessage("WATCHDOG: Arm stopped!");
  }
}

void enableWatchdog() {
  watchdog_enabled = true;
  last_base_cmd_time = millis();
  last_arm_cmd_time = millis();
  displayMessage("Watchdog enabled");
}

bool initMicroROS(){
    displayMessage("Starting microROS");
    set_microros_serial_transports(Serial);
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    displayMessage("Allocator started");

    // Create message memory for dynamic arrays
    micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      &base_cmd_msg,
      (micro_ros_utilities_memory_conf_t) {
        .max_string_capacity = 0,
        .max_ros2_type_sequence_capacity = BASE_SERVO_COUNT,
        .max_basic_type_sequence_capacity = BASE_SERVO_COUNT
      });

    micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      &arm_cmd_msg,
      (micro_ros_utilities_memory_conf_t) {
        .max_string_capacity = 0,
        .max_ros2_type_sequence_capacity = ARM_SERVO_COUNT,
        .max_basic_type_sequence_capacity = ARM_SERVO_COUNT
      });

    micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      &joint_state_msg,
      (micro_ros_utilities_memory_conf_t) {
        .max_string_capacity = 20,  // For joint names
        .max_ros2_type_sequence_capacity = BASE_SERVO_COUNT + ARM_SERVO_COUNT,
        .max_basic_type_sequence_capacity = BASE_SERVO_COUNT + ARM_SERVO_COUNT
      });


    // Initialize joint names for joint state message
    joint_state_msg.name.data[0] = micro_ros_string_utilities_set(joint_state_msg.name.data[0], "back_motor_rotation");
    joint_state_msg.name.data[1] = micro_ros_string_utilities_set(joint_state_msg.name.data[1], "left_motor_rotation");
    joint_state_msg.name.data[2] = micro_ros_string_utilities_set(joint_state_msg.name.data[2], "right_motor_rotation");
    joint_state_msg.name.data[3] = micro_ros_string_utilities_set(joint_state_msg.name.data[3], "1");
    joint_state_msg.name.data[4] = micro_ros_string_utilities_set(joint_state_msg.name.data[4], "2");
    joint_state_msg.name.data[5] = micro_ros_string_utilities_set(joint_state_msg.name.data[5], "3");
    joint_state_msg.name.data[6] = micro_ros_string_utilities_set(joint_state_msg.name.data[6], "4");
    joint_state_msg.name.data[7] = micro_ros_string_utilities_set(joint_state_msg.name.data[7], "5");
    joint_state_msg.name.data[8] = micro_ros_string_utilities_set(joint_state_msg.name.data[8], "6");
    joint_state_msg.name.data[9] = micro_ros_string_utilities_set(joint_state_msg.name.data[9], "camera_tilt");
    joint_state_msg.name.size = BASE_SERVO_COUNT + ARM_SERVO_COUNT;

    // Create node
    rcl_ret_t rc = rclc_node_init_default(
      &node,
      "esp32_bridge_node",
      "",
      &support
    );

    if (rc != RCL_RET_OK) {
      displayMessage("Error creating micro-ROS node!");
      return false;
    } else {
      displayMessage("Successfully created micro-ROS node");
    }

    // Create executor with 2 subscribers
    rclc_executor_init(&executor, &support.context, 2, &allocator);

    // Initialize Base Command Subscriber
    rclc_subscription_init_default(
      &base_cmd_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      BASE_CMD_TOPIC
    );

    rc = rclc_executor_add_subscription(
      &executor,
      &base_cmd_sub,
      &base_cmd_msg,
      &base_cmd_callback,
      ON_NEW_DATA
    );

    if (rc != RCL_RET_OK) {
      displayMessage("Error adding base cmd subscription!");
      return false;
    } else {
      displayMessage("Successfully added base cmd subscription");
    }

    // Initialize Arm Command Subscriber
    rclc_subscription_init_default(
      &arm_cmd_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      ARM_CMD_TOPIC
    );

    rc = rclc_executor_add_subscription(
      &executor,
      &arm_cmd_sub,
      &arm_cmd_msg,
      &arm_cmd_callback,
      ON_NEW_DATA
    );

    if (rc != RCL_RET_OK) {
      displayMessage("Error adding arm cmd subscription!");
      return false;
    } else {
      displayMessage("Successfully added arm cmd subscription");
    }

    // Initialize Joint State Publisher
    rc = rclc_publisher_init_default(
      &joint_state_pub, 
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      JOINT_STATE_TOPIC
    );

    if(rc != RCL_RET_OK) {
      displayMessage("Joint state publisher failed");
      return false;
    } else {
      displayMessage("Successfully added joint state publisher");
    }

    displayMessage("microROS initialized!");
    return true;
}

void spinMicroRos()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}

void moveToRelaxedPositions() {
  displayMessage("Moving to relaxed positions...");
  
  // Move arm servos to their calibrated relaxed positions
  for (int i = 0; i < ARM_CALIBRATION_COUNT; i++) {
    const ServoCalibration& cal = ARM_CALIBRATION[i];
    arm_servo.WritePosEx(cal.servo_id, cal.relaxed_position, 500, 0); // 500ms time to reach position
    delay(100); // Small delay between commands
    
    char buf[64];
    sprintf(buf, "Servo %d -> %d", cal.servo_id, cal.relaxed_position);
    displayMessage(buf);
    delay(300);
  }
  
  displayMessage("Relaxed position set!");
  delay(1000);
}

// Convert servo encoder position to radians using calibration data
float servoPositionToRadians(uint8_t servo_id, int32_t position) {
  // Find calibration data for this servo
  for (int i = 0; i < ARM_CALIBRATION_COUNT; i++) {
    if (ARM_CALIBRATION[i].servo_id == servo_id) {
      const ServoCalibration& cal = ARM_CALIBRATION[i];
      // Map position from [min_pos, max_pos] to [-PI, PI] range
      float normalized = (float)(position - cal.min_position) / (float)(cal.max_position - cal.min_position);
      return (normalized * 2.0 * PI) - PI; // Convert to -PI to PI range
    }
  }
  // Fallback to old calculation if calibration not found
  return ((position - 2048) * 2.0 * PI) / 4096.0;
}

// Convert radians to servo encoder position using calibration data
int16_t radiansToServoPosition(uint8_t servo_id, float radians) {
  // Find calibration data for this servo
  for (int i = 0; i < ARM_CALIBRATION_COUNT; i++) {
    if (ARM_CALIBRATION[i].servo_id == servo_id) {
      const ServoCalibration& cal = ARM_CALIBRATION[i];
      // Map radians from [-PI, PI] to [min_pos, max_pos] range
      float normalized = (radians + PI) / (2.0 * PI); // Convert to 0-1 range
      int16_t position = cal.min_position + (int16_t)(normalized * (cal.max_position - cal.min_position));
      return constrain(position, cal.min_position, cal.max_position);
    }
  }
  // Fallback to old calculation if calibration not found
  return constrain(2048 + (radians * 651.74), 0, 4095);
}

void publishJointStates() {
  // Read current positions from all servos
  joint_state_msg.position.size = BASE_SERVO_COUNT + ARM_SERVO_COUNT;
  joint_state_msg.velocity.size = BASE_SERVO_COUNT + ARM_SERVO_COUNT;
  joint_state_msg.effort.size = 0;  // Not using effort feedback
  
  // Update timestamp
  int64_t timestamp = rmw_uros_epoch_nanos();
  joint_state_msg.header.stamp.sec = timestamp / 1000000000ULL;
  joint_state_msg.header.stamp.nanosec = timestamp % 1000000000ULL;
  
  // Read base servo positions (convert to radians)
  for (int i = 0; i < BASE_SERVO_COUNT; i++) {
    int32_t pos = base_servo.ReadPos(BASE_SERVO_IDS[i]);
    if (pos != -1) {
      // Convert servo position to radians (for wheels, this is cumulative rotation)
      joint_state_msg.position.data[i] = (pos * 2.0 * PI) / 4096.0;
    } else {
      joint_state_msg.position.data[i] = 0.0;  // Default if read fails
    }
    
    int16_t speed = base_servo.ReadSpeed(BASE_SERVO_IDS[i]);
    if (speed != -1) {
      // Convert servo speed to rad/s
      joint_state_msg.velocity.data[i] = speed / VEL_TO_SERVO_UNIT;
    } else {
      joint_state_msg.velocity.data[i] = 0.0;
    }
  }
  
  // Read arm servo positions (convert to radians using calibration)
  for (int i = 0; i < ARM_SERVO_COUNT; i++) {
    int32_t pos = arm_servo.ReadPos(ARM_SERVO_IDS[i]);
    if (pos != -1) {
      // Use calibrated conversion function
      joint_state_msg.position.data[BASE_SERVO_COUNT + i] = servoPositionToRadians(ARM_SERVO_IDS[i], pos);
    } else {
      joint_state_msg.position.data[BASE_SERVO_COUNT + i] = 0.0;  // Default if read fails
    }
    
    int16_t speed = arm_servo.ReadSpeed(ARM_SERVO_IDS[i]);
    if (speed != -1) {
      // Convert servo speed to rad/s  
      joint_state_msg.velocity.data[BASE_SERVO_COUNT + i] = (speed * 2.0 * PI) / (4096.0 * 60.0);  // Assuming speed in RPM
    } else {
      joint_state_msg.velocity.data[BASE_SERVO_COUNT + i] = 0.0;
    }
  }
  
  // Publish joint states
  rcl_publish(&joint_state_pub, &joint_state_msg, NULL);
}



bool checkServos() {
  displayMessage("Checking base servos...");
  delay(500);

  // Check base servos (wheel motors)
  for (int i = 0; i < BASE_SERVO_COUNT; i++) {
    uint8_t servo_id = BASE_SERVO_IDS[i];
    int pingStatus = base_servo.Ping(servo_id);
    if (pingStatus == -1) {
      char buf[64];
      sprintf(buf, "Base servo %d FAIL!", servo_id);
      displayMessage(buf);
      delay(1000);
    } else {
      char buf[64];
      base_servo.WheelMode(servo_id);  // Set to continuous rotation mode
      base_servo.EnableTorque(servo_id, 1);
      sprintf(buf, "Base servo %d OK", servo_id);
      displayMessage(buf);
      delay(500);
    }
  }

  displayMessage("Checking arm servos...");
  delay(500);

  // Check arm servos (position control)
  for (int i = 0; i < ARM_SERVO_COUNT; i++) {
    uint8_t servo_id = ARM_SERVO_IDS[i];
    int pingStatus = arm_servo.Ping(servo_id);
    if (pingStatus == -1) {
      char buf[64];
      sprintf(buf, "Arm servo %d FAIL!", servo_id);
      displayMessage(buf);
      delay(1000);
    } else {
      char buf[64];
      // Servos should be in position mode by default
      arm_servo.EnableTorque(servo_id, 1);
      sprintf(buf, "Arm servo %d OK", servo_id);
      displayMessage(buf);
      delay(500);
    }
  }

  displayMessage("All servos OK!");
  delay(500);
  return true;
}

void setup() {
  initScreen();
  displayMessage("Board started");
  delay(1000);

  // Initialize serial communication for servos
  Serial2.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  base_servo.pSerial = &Serial2;  // Base servos use SMS_STS (continuous rotation)
  arm_servo.pSerial = &Serial2;   // Arm servos use SCSCL (position control)

  checkServos();
  
  // Move arm to relaxed positions after servo check
  moveToRelaxedPositions();

  // Initialize micro-ROS serial communication
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  if (!initMicroROS()) {
    displayMessage("micro-ROS initialization failed!");
  } else {
    // Enable watchdog after successful ROS connection
    delay(1000);  // Give ROS connection time to stabilize
    enableWatchdog();
  }
}

void loop() {
  // Run micro-ROS executor to handle incoming commands
  spinMicroRos();

  // Check watchdog for safety
  checkWatchdog();

  // Publish joint states at 200Hz (every 5ms)
  if(millis() - last_joint_state_time >= CONTROL_PERIOD_MS){
    publishJointStates();
    last_joint_state_time = millis();
  }
  
  // Small delay to prevent overwhelming the system
  delayMicroseconds(100);
}