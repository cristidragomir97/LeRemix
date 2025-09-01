#include "microros.h"
#include "utils.h"
#include "config.h"

// Define the global variables
rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

bool initMicroROS(void* display) {
    set_microros_serial_transports(Serial);
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    // Create node
    rcl_ret_t rc = rclc_node_init_default(
      &node,
      "esp32_bridge_node",
      "",
      &support
    );

    if (rc != RCL_RET_OK) {
        Serial1.println("Error creating micro-ROS node!");
        return false;
    } else {
        Serial1.println("Successfully created micro-ROS node");
    }

    // Create executor with 2 subscribers
    rclc_executor_init(&executor, &support.context, 2, &allocator);

    Serial1.println("micro-ROS node and basic setup ready");
    return true;
}

void spinMicroRos() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
}