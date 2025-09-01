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
static Adafruit_SSD1306* display_ptr = nullptr;
static QueueHandle_t arm_command_queue_handle = nullptr;

// Arm command message structure
struct ArmCommandMsg {
  float joint_positions[7];  // 6 arm joints + 1 camera
  int joint_count;
};

bool initArmSubscriber(rcl_node_t* node, rclc_executor_t* executor, Adafruit_SSD1306* display) {
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
    rcl_ret_t rc = rclc_subscription_init_default(
      &arm_cmd_sub,
      node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      ARM_CMD_TOPIC
    );

    if (rc != RCL_RET_OK) {
        displayMessage("Error creating arm cmd subscription!", display);
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
        displayMessage("Error adding arm cmd subscription!", display);
        return false;
    }

    displayMessage("Successfully added arm cmd subscription", display);
    return true;
}

void setArmServosEnabledPtr(bool* servos_enabled_ptr_arg) {
    servos_enabled_ptr = servos_enabled_ptr_arg;
}

void setArmDisplayPtr(Adafruit_SSD1306* display_ptr_arg) {
    display_ptr = display_ptr_arg;
}

void setArmCommandQueue(QueueHandle_t queue_handle) {
    arm_command_queue_handle = queue_handle;
}

// Arm command callback - expects Float64MultiArray with 7 values (radians)
void arm_cmd_callback(const void *msgin) {
    const std_msgs__msg__Float64MultiArray *msg = (std_msgs__msg__Float64MultiArray *)msgin;
    
    if (msg->data.size >= ARM_SERVO_COUNT && arm_command_queue_handle != nullptr) {
        ArmCommandMsg arm_cmd;
        arm_cmd.joint_count = min((int)msg->data.size, 7);
        
        // Copy joint positions
        for (int i = 0; i < arm_cmd.joint_count; i++) {
            arm_cmd.joint_positions[i] = msg->data.data[i];
        }
        
        // Send command to servo control task via queue
        // Non-blocking send - if queue is full, drop the message
        xQueueSend(arm_command_queue_handle, &arm_cmd, 0);
        
        // Update display if available
        if (display_ptr) {
            char buffer[64];
            snprintf(buffer, sizeof(buffer), "Arm: %d joints", arm_cmd.joint_count);
            // displayMessage(buffer, display_ptr);  // Commented out to avoid flooding display
        }
    }
}