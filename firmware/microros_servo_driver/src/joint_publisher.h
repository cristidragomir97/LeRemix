#ifndef JOINT_PUBLISHER_H
#define JOINT_PUBLISHER_H

#include <rcl/rcl.h>
#include <sensor_msgs/msg/joint_state.h>
#include <Adafruit_SSD1306.h>

// Joint state publisher objects
extern rcl_publisher_t joint_state_pub;
extern sensor_msgs__msg__JointState joint_state_msg;

// Control timing
extern unsigned long last_joint_state_time;
extern const unsigned long CONTROL_PERIOD_MS;  // 200Hz = 5ms period

// Initialize joint state publisher
bool initJointPublisher(rcl_node_t* node, Adafruit_SSD1306* display);

// Publish joint states
void publishJointStates();

// Check if it's time to publish joint states
bool shouldPublishJointStates();

#endif