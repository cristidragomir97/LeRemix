#ifndef MICROROS_H
#define MICROROS_H

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rmw_microros/rmw_microros.h>
#include <Adafruit_SSD1306.h>

// Topic names matching leremix_control_plugin
#define BASE_CMD_TOPIC   "/esp32/base_cmd"
#define ARM_CMD_TOPIC    "/esp32/arm_cmd"
#define JOINT_STATE_TOPIC "/esp32/joint_states"

// Global micro-ROS objects
extern rcl_node_t node;
extern rclc_executor_t executor;
extern rclc_support_t support;
extern rcl_allocator_t allocator;

// Initialize micro-ROS system
bool initMicroROS(Adafruit_SSD1306* display);

// Spin the executor
void spinMicroRos();

#endif