#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <sensor_msgs/msg/joint_state.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rmw_microros/rmw_microros.h>

// WiFi credentials - CHANGE THESE TO YOUR NETWORK
const char* ssid = "your_wifi_ssid";        // Replace with your WiFi network name
const char* password = "your_wifi_password"; // Replace with your WiFi password

// Web server
WebServer server(80);

// Topic names matching leremix_control_plugin
#define BASE_CMD_TOPIC   "/esp32/base_cmd"
#define ARM_CMD_TOPIC    "/esp32/arm_cmd"
#define JOINT_STATE_TOPIC "/esp32/joint_states"

// ROS configuration
#define BASE_SERVO_COUNT 3
#define ARM_SERVO_COUNT 7

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

// Logging
String log_buffer = "";
unsigned long last_joint_state_time = 0;
const unsigned long CONTROL_PERIOD_MS = 50;  // 20Hz for testing

// Statistics
unsigned long base_msg_count = 0;
unsigned long arm_msg_count = 0;
unsigned long joint_state_msg_count = 0;

void addLog(String message) {
  String timestamp = String(millis());
  String logEntry = "[" + timestamp + "] " + message + "\n";
  Serial.print(logEntry);
  
  // Keep only last 2000 characters to prevent memory issues
  log_buffer += logEntry;
  if (log_buffer.length() > 2000) {
    log_buffer = log_buffer.substring(log_buffer.length() - 1500);
  }
}

// Base command callback
void base_cmd_callback(const void *msgin) {
  const std_msgs__msg__Float64MultiArray *msg = (std_msgs__msg__Float64MultiArray *)msgin;
  base_msg_count++;
  
  String logMsg = "BASE_CMD: size=" + String(msg->data.size) + " capacity=" + String(msg->data.capacity) + " [";
  
  if (msg->data.data == NULL) {
    logMsg += "NULL_DATA";
  } else {
    for (size_t i = 0; i < msg->data.size && i < BASE_SERVO_COUNT; i++) {
      logMsg += String(msg->data.data[i], 6);  // More precision
      if (i < msg->data.size - 1) logMsg += ", ";
    }
  }
  logMsg += "]";
  
  addLog(logMsg);
}

// Arm command callback
void arm_cmd_callback(const void *msgin) {
  const std_msgs__msg__Float64MultiArray *msg = (std_msgs__msg__Float64MultiArray *)msgin;
  arm_msg_count++;
  
  String logMsg = "ARM_CMD: size=" + String(msg->data.size) + " capacity=" + String(msg->data.capacity) + " [";
  
  if (msg->data.data == NULL) {
    logMsg += "NULL_DATA";
  } else {
    for (size_t i = 0; i < msg->data.size && i < ARM_SERVO_COUNT; i++) {
      logMsg += String(msg->data.data[i], 6);  // More precision
      if (i < msg->data.size - 1) logMsg += ", ";
    }
  }
  logMsg += "]";
  
  addLog(logMsg);
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<title>ESP32 ROS Debug Monitor</title>";
  html += "<meta http-equiv='refresh' content='2'>";
  html += "</head><body>";
  html += "<h1>ESP32 ROS Communication Test</h1>";
  html += "<h2>Statistics</h2>";
  html += "<p>Uptime: " + String(millis()/1000) + " seconds</p>";
  html += "<p>Base Commands Received: " + String(base_msg_count) + "</p>";
  html += "<p>Arm Commands Received: " + String(arm_msg_count) + "</p>";
  html += "<p>Joint States Published: " + String(joint_state_msg_count) + "</p>";
  html += "<p>WiFi RSSI: " + String(WiFi.RSSI()) + " dBm</p>";
  html += "<h2>Recent Messages</h2>";
  html += "<pre style='background-color: #f0f0f0; padding: 10px; font-family: monospace; white-space: pre-wrap; word-wrap: break-word;'>";
  html += log_buffer;
  html += "</pre>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

void handleClear() {
  log_buffer = "";
  base_msg_count = 0;
  arm_msg_count = 0;
  joint_state_msg_count = 0;
  addLog("Log cleared via web interface");
  server.send(200, "text/plain", "Log cleared. Go back to main page.");
}

bool initMicroROS() {
  addLog("Initializing micro-ROS...");
  
  IPAddress agent_ip(192, 168, 1, 100);  // CHANGE THIS TO YOUR ROS2 MACHINE IP
  size_t agent_port = 8888;               // Default micro-ROS agent port
  
  char agent_ip_str[16];
  sprintf(agent_ip_str, "%d.%d.%d.%d", agent_ip[0], agent_ip[1], agent_ip[2], agent_ip[3]);
  
  set_microros_wifi_transports((char*)ssid, (char*)password, agent_ip_str, agent_port);
  
  addLog("Connecting to micro-ROS agent at " + String(agent_ip_str) + ":" + String(agent_port));
  
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  
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
  rcl_ret_t rc = rclc_node_init_default(&node, "esp32_debug_node", "", &support);
  if (rc != RCL_RET_OK) {
    addLog("ERROR: Failed to create micro-ROS node");
    return false;
  }
  addLog("Created micro-ROS node");

  // Create executor with 2 subscribers
  rclc_executor_init(&executor, &support.context, 2, &allocator);

  // Initialize Base Command Subscriber with best-effort QoS
  rclc_subscription_init_best_effort(
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
    addLog("ERROR: Failed to add base cmd subscription");
    return false;
  }
  addLog("Added base command subscription: " + String(BASE_CMD_TOPIC));

  // Initialize Arm Command Subscriber with best-effort QoS
  rclc_subscription_init_best_effort(
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
    addLog("ERROR: Failed to add arm cmd subscription");
    return false;
  }
  addLog("Added arm command subscription: " + String(ARM_CMD_TOPIC));

  // Initialize Joint State Publisher with best-effort QoS
  rc = rclc_publisher_init_best_effort(
    &joint_state_pub, 
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    JOINT_STATE_TOPIC
  );

  if(rc != RCL_RET_OK) {
    addLog("ERROR: Failed to create joint state publisher");
    return false;
  }
  addLog("Created joint state publisher: " + String(JOINT_STATE_TOPIC));

  addLog("micro-ROS initialization complete!");
  return true;
}

void publishJointStates() {
  // Update timestamp
  int64_t timestamp = rmw_uros_epoch_nanos();
  joint_state_msg.header.stamp.sec = timestamp / 1000000000ULL;
  joint_state_msg.header.stamp.nanosec = timestamp % 1000000000ULL;
  
  // Set frame_id
  joint_state_msg.header.frame_id = micro_ros_string_utilities_set(joint_state_msg.header.frame_id, "");
  
  // Set sizes
  joint_state_msg.position.size = BASE_SERVO_COUNT + ARM_SERVO_COUNT;
  joint_state_msg.velocity.size = BASE_SERVO_COUNT + ARM_SERVO_COUNT;
  joint_state_msg.effort.size = 0;
  
  // Check if arrays are properly allocated
  if (joint_state_msg.position.data == NULL || joint_state_msg.velocity.data == NULL) {
    addLog("ERROR: Joint state message arrays not allocated!");
    return;
  }
  
  // Generate some test data (simulated joint positions)
  for (int i = 0; i < BASE_SERVO_COUNT + ARM_SERVO_COUNT; i++) {
    joint_state_msg.position.data[i] = sin(millis() / 1000.0 + i) * 0.5;  // Oscillating positions
    joint_state_msg.velocity.data[i] = cos(millis() / 1000.0 + i) * 0.1;  // Oscillating velocities
  }
  
  // Publish joint states
  rcl_ret_t rc = rcl_publish(&joint_state_pub, &joint_state_msg, NULL);
  if (rc == RCL_RET_OK) {
    joint_state_msg_count++;
    if (joint_state_msg_count % 20 == 0) {  // Log every 20th message (once per second)
      addLog("Published joint state #" + String(joint_state_msg_count) + " (pos[0]=" + String(joint_state_msg.position.data[0], 3) + ")");
    }
  } else {
    addLog("ERROR: Failed to publish joint state, rc=" + String(rc) + " (" + getErrorMessage(rc) + ")");
  }
}

String getErrorMessage(rcl_ret_t rc) {
  switch (rc) {
    case RCL_RET_OK: return "OK";
    case RCL_RET_ERROR: return "GENERIC_ERROR";
    case RCL_RET_BAD_ALLOC: return "BAD_ALLOC";
    case RCL_RET_INVALID_ARGUMENT: return "INVALID_ARGUMENT";
    case RCL_RET_NOT_INIT: return "NOT_INIT";
    case RCL_RET_PUBLISHER_INVALID: return "PUBLISHER_INVALID";
    case RCL_RET_TIMEOUT: return "TIMEOUT";
    default: return "UNKNOWN_" + String(rc);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 ROS Debug Monitor Starting...");
  
  // Connect to WiFi
  addLog("Connecting to WiFi: " + String(ssid));
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  
  addLog("WiFi connected! IP: " + WiFi.localIP().toString());
  
  // Setup web server
  server.on("/", handleRoot);
  server.on("/clear", handleClear);
  server.begin();
  addLog("Web server started on port 80");
  
  // Initialize micro-ROS
  bool ros_ok = initMicroROS();
  if (!ros_ok) {
    addLog("micro-ROS initialization failed! Check agent connection.");
  }
  
  addLog("Setup complete. Open http://" + WiFi.localIP().toString() + " in your browser");
}

void loop() {
  // Handle web server
  server.handleClient();
  
  // Run micro-ROS executor
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  
  // Publish joint states periodically
  if (millis() - last_joint_state_time >= CONTROL_PERIOD_MS) {
    publishJointStates();
    last_joint_state_time = millis();
  }
  
  delay(1);
}