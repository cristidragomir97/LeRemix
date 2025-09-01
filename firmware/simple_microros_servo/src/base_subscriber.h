#ifndef BASE_SUBSCRIBER_H
#define BASE_SUBSCRIBER_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64_multi_array.h>

// Base subscriber objects
extern rcl_subscription_t base_cmd_sub;
extern std_msgs__msg__Float64MultiArray base_cmd_msg;

// Initialize base subscriber
bool initBaseSubscriber(rcl_node_t* node, rclc_executor_t* executor, void* display);

// Base command callback
void base_cmd_callback(const void *msgin);

// Set servos enabled state pointer for callback use
void setBaseServosEnabledPtr(bool* servos_enabled_ptr);

// Set display pointer for callback use (unused in simplified version)
void setBaseDisplayPtr(void* display_ptr);

#endif