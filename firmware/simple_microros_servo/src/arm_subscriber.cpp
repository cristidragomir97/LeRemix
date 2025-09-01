#include "arm_subscriber.h"
#include "microros.h"
#include "motor_controller.h"
#include "utils.h"
#include "config.h"
#include <micro_ros_utilities/type_utilities.h>

// Arm subscriber objects
rcl_subscription_t arm_cmd_sub;
std_msgs__msg__Float64MultiArray arm_cmd_msg;

// Pointers to external variables
static bool* servos_enabled_ptr = nullptr;

bool initArmSubscriber(rcl_node_t* node, rclc_executor_t* executor, void* display) {
    // Create message memory for dynamic arrays
    micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      &arm_cmd_msg,
      (micro_ros_utilities_memory_conf_t) {
        .max_string_capacity = 0,
        .max_ros2_type_sequence_capacity = ARM_SERVO_COUNT,
        .max_basic_type_sequence_capacity = ARM_SERVO_COUNT
      });

    // Initialize Arm Command Subscriber with best effort QoS for ros2_control compatibility
    rcl_ret_t rc = rclc_subscription_init_best_effort(
      &arm_cmd_sub,
      node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      ARM_CMD_TOPIC
    );

    if (rc != RCL_RET_OK) {
        Serial.println("Error creating arm cmd subscription!");
        return false;
    }

    rc = rclc_executor_add_subscription(
      executor,
      &arm_cmd_sub,
      &arm_cmd_msg,
      &arm_cmd_callback,
      ON_NEW_DATA
    );

    if (rc != RCL_RET_OK) {
        Serial.println("Error adding arm cmd subscription!");
        return false;
    }

    Serial.println("Successfully added arm cmd subscription");
    return true;
}

void setArmServosEnabledPtr(bool* servos_enabled_ptr_arg) {
    servos_enabled_ptr = servos_enabled_ptr_arg;
}

void setArmDisplayPtr(void* display_ptr_arg) {
    // Display functionality removed in simplified version
}

// Arm command callback - expects Float64MultiArray with 7 values (radians)
void arm_cmd_callback(const void *msgin) {
    const std_msgs__msg__Float64MultiArray *msg = (std_msgs__msg__Float64MultiArray *)msgin;
    
    #if ENABLE_DEBUG_PRINTS
    Serial.println("=== ARM COMMAND CALLBACK ===");
    Serial.printf("Message data size: %d\n", msg->data.size);
    if (servos_enabled_ptr) {
        Serial.printf("Servos enabled: %s\n", *servos_enabled_ptr ? "YES" : "NO");
    }
    #endif
    
    if (msg->data.size >= ARM_SERVO_COUNT) {
        #if ENABLE_DEBUG_PRINTS
        Serial.print("Raw values: [");
        for (int i = 0; i < ARM_SERVO_COUNT; i++) {
            Serial.printf("%.4f", msg->data.data[i]);
            if (i < ARM_SERVO_COUNT - 1) Serial.print(", ");
        }
        Serial.println("]");
        #endif
        
        // Convert radians to servo position units and send to arm servos
        for (int i = 0; i < ARM_SERVO_COUNT; i++) {
            float radians = msg->data.data[i];
            bool enabled = servos_enabled_ptr ? *servos_enabled_ptr : false;
            controlArmServo(i, radians, enabled);
        }
        
        // Simplified - no display output
    } else {
        #if ENABLE_DEBUG_PRINTS
        Serial.printf("ERROR: Insufficient data size (%d < %d)\n", msg->data.size, ARM_SERVO_COUNT);
        #endif
    }
}