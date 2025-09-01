#include "base_subscriber.h"
#include "microros.h"
#include "motor_controller.h"
#include "utils.h"
#include "config.h"
#include "debug_serial.h"
#include <micro_ros_utilities/type_utilities.h>

// Base subscriber objects
rcl_subscription_t base_cmd_sub;
std_msgs__msg__Float64MultiArray base_cmd_msg;

// Pointers to external variables
static bool* servos_enabled_ptr = nullptr;

// Message deduplication - store last received values
static float last_base_values[BASE_SERVO_COUNT] = {0};
static bool first_base_message = true;

bool initBaseSubscriber(rcl_node_t* node, rclc_executor_t* executor, void* display) {
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
        Serial1.println("Error creating base cmd subscription!");
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
        Serial1.println("Error adding base cmd subscription!");
        return false;
    }

    Serial1.println("Successfully added base cmd subscription");
    return true;
}

void setBaseServosEnabledPtr(bool* servos_enabled_ptr_arg) {
    servos_enabled_ptr = servos_enabled_ptr_arg;
}

void setBaseDisplayPtr(void* display_ptr_arg) {
    // Display functionality removed in simplified version
}

// Base command callback - expects Float64MultiArray with 3 values (rad/s)
void base_cmd_callback(const void *msgin) {
   // DEBUG_CALLBACK_START("BASE");
    const std_msgs__msg__Float64MultiArray *msg = (std_msgs__msg__Float64MultiArray *)msgin;
    
    #if ENABLE_DEBUG_PRINTS
    Serial1.println("=== BASE COMMAND CALLBACK ===");
    Serial.printf("Message data size: %d\n", msg->data.size);
    if (servos_enabled_ptr) {
        Serial.printf("Servos enabled: %s\n", *servos_enabled_ptr ? "YES" : "NO");
    }
    #endif
    
    if (msg->data.size >= BASE_SERVO_COUNT) {
        // Check if values have changed (skip processing if identical)
        bool values_changed = first_base_message;
        if (!first_base_message) {
            for (int i = 0; i < BASE_SERVO_COUNT; i++) {
                if (abs(msg->data.data[i] - last_base_values[i]) > 0.01) { // 0.01 rad/s tolerance
                    values_changed = true;
                    break;
                }
            }
        }
        
        if (values_changed) {
            #if ENABLE_DEBUG_PRINTS
            Serial.printf("Raw values: [%.4f, %.4f, %.4f]\n", 
                          msg->data.data[0], msg->data.data[1], msg->data.data[2]);
            #endif
            
            // Store new values and send commands
            for (int i = 0; i < BASE_SERVO_COUNT; i++) {
                float rad_per_sec = msg->data.data[i];
                last_base_values[i] = rad_per_sec;
                bool enabled = servos_enabled_ptr ? *servos_enabled_ptr : false;
                controlBaseServo(i, rad_per_sec, enabled);
            }
            
            first_base_message = false;
        }
        #if ENABLE_DEBUG_PRINTS
        else {
            Serial1.println("BASE: Identical message, skipping servo commands");
        }
        #endif
        
        // Simplified - no display output
    } else {
        #if ENABLE_DEBUG_PRINTS
        Serial.printf("ERROR: Insufficient data size (%d < %d)\n", msg->data.size, BASE_SERVO_COUNT);
        #endif
    }
    //DEBUG_CALLBACK_END("BASE");
}