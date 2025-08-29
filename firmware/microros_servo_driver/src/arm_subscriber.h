#ifndef ARM_SUBSCRIBER_H
#define ARM_SUBSCRIBER_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <Adafruit_SSD1306.h>

// Arm subscriber objects
extern rcl_subscription_t arm_cmd_sub;
extern std_msgs__msg__Float64MultiArray arm_cmd_msg;

// Initialize arm subscriber
bool initArmSubscriber(rcl_node_t* node, rclc_executor_t* executor, Adafruit_SSD1306* display);

// Arm command callback
void arm_cmd_callback(const void *msgin);

// Set servos enabled state pointer for callback use
void setArmServosEnabledPtr(bool* servos_enabled_ptr);

// Set display pointer for callback use
void setArmDisplayPtr(Adafruit_SSD1306* display_ptr);

#endif