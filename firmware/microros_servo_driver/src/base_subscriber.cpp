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
static QueueHandle_t base_command_queue_handle = nullptr;

// Base command message structure
struct BaseCommandMsg {
  float linear_x;
  float angular_z;
};

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

void setBaseCommandQueue(QueueHandle_t queue_handle) {
    base_command_queue_handle = queue_handle;
}

// Base command callback - expects Float64MultiArray with 2 values (linear_x, angular_z)
void base_cmd_callback(const void *msgin) {
    const std_msgs__msg__Float64MultiArray *msg = (std_msgs__msg__Float64MultiArray *)msgin;
    
    if (msg->data.size >= 2 && base_command_queue_handle != nullptr) {
        BaseCommandMsg base_cmd;
        base_cmd.linear_x = msg->data.data[0];
        base_cmd.angular_z = msg->data.data[1];
        
        // Send command to servo control task via queue
        // Non-blocking send - if queue is full, drop the message
        xQueueSend(base_command_queue_handle, &base_cmd, 0);
        
        // Update display if available
        if (display_ptr) {
            char buffer[64];
            snprintf(buffer, sizeof(buffer), "Base: %.2f, %.2f", base_cmd.linear_x, base_cmd.angular_z);
            // displayMessage(buffer, display_ptr);  // Commented out to avoid flooding display
        }
    }
}