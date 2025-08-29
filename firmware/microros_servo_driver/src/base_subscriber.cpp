#include "base_subscriber.h"
#include "microros.h"
#include "motor_controller.h"
#include "utils.h"
#include "config.h"
#include <micro_ros_utilities/type_utilities.h>

// Base subscriber objects
rcl_subscription_t base_cmd_sub;
std_msgs__msg__Float64MultiArray base_cmd_msg;

// Pointers to external variables
static bool* servos_enabled_ptr = nullptr;
static Adafruit_SSD1306* display_ptr = nullptr;

bool initBaseSubscriber(rcl_node_t* node, rclc_executor_t* executor, Adafruit_SSD1306* display) {
    // Create message memory for dynamic arrays
    micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      &base_cmd_msg,
      (micro_ros_utilities_memory_conf_t) {
        .max_string_capacity = 0,
        .max_ros2_type_sequence_capacity = BASE_SERVO_COUNT,
        .max_basic_type_sequence_capacity = BASE_SERVO_COUNT
      });

    // Initialize Base Command Subscriber with best effort QoS for ros2_control compatibility
    rcl_ret_t rc = rclc_subscription_init_best_effort(
      &base_cmd_sub,
      node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      BASE_CMD_TOPIC
    );

    if (rc != RCL_RET_OK) {
        displayMessage("Error creating base cmd subscription!", display);
        return false;
    }

    rc = rclc_executor_add_subscription(
      executor,
      &base_cmd_sub,
      &base_cmd_msg,
      &base_cmd_callback,
      ON_NEW_DATA
    );

    if (rc != RCL_RET_OK) {
        displayMessage("Error adding base cmd subscription!", display);
        return false;
    }

    displayMessage("Successfully added base cmd subscription", display);
    return true;
}

void setBaseServosEnabledPtr(bool* servos_enabled_ptr_arg) {
    servos_enabled_ptr = servos_enabled_ptr_arg;
}

void setBaseDisplayPtr(Adafruit_SSD1306* display_ptr_arg) {
    display_ptr = display_ptr_arg;
}

// Base command callback - expects Float64MultiArray with 3 values (rad/s)
void base_cmd_callback(const void *msgin) {
    const std_msgs__msg__Float64MultiArray *msg = (std_msgs__msg__Float64MultiArray *)msgin;
    
    if (display_ptr) {
        displayMessage("Base command received!", display_ptr);
    }
    
    // Display callback info on screen
    if (display_ptr) {
        char debug_buf[128];
        sprintf(debug_buf, "BASE CB: size=%d", msg->data.size);
        displayMessage(debug_buf, display_ptr);
        delay(500); // Brief display of callback info
    }
    
    #if ENABLE_DEBUG_PRINTS
    Serial.println("=== BASE COMMAND CALLBACK ===");
    Serial.printf("Message data size: %d\n", msg->data.size);
    if (servos_enabled_ptr) {
        Serial.printf("Servos enabled: %s\n", *servos_enabled_ptr ? "YES" : "NO");
    }
    #endif
    
    if (msg->data.size >= BASE_SERVO_COUNT) {
        #if ENABLE_DEBUG_PRINTS
        Serial.printf("Raw values: [%.4f, %.4f, %.4f]\n", 
                      msg->data.data[0], msg->data.data[1], msg->data.data[2]);
        #endif
        
        // Convert rad/s to servo speed units and send to base servos
        for (int i = 0; i < BASE_SERVO_COUNT; i++) {
            float rad_per_sec = msg->data.data[i];
            bool enabled = servos_enabled_ptr ? *servos_enabled_ptr : false;
            controlBaseServo(i, rad_per_sec, enabled);
        }
        
        // Update display if available
        if (display_ptr) {
            char buf[128];
            sprintf(buf, "Base: %.2f %.2f %.2f", 
                    msg->data.data[0], msg->data.data[1], msg->data.data[2]);
            displayMessage(buf, display_ptr);
        }
    } else {
        #if ENABLE_DEBUG_PRINTS
        Serial.printf("ERROR: Insufficient data size (%d < %d)\n", msg->data.size, BASE_SERVO_COUNT);
        #endif
    }
}