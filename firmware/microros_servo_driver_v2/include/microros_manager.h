#pragma once

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <sensor_msgs/msg/joint_state.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rmw_microros/rmw_microros.h>
#include "config.h"

// Micro-ROS communication manager
class MicroROSManager {
public:
    MicroROSManager();
    void begin();
    void update();
    void publishJointStates();
    
    bool isInitialized() const { return initialized_; }
    
    // Statistics
    unsigned long getBaseMessageCount() const { return base_msg_count_; }
    unsigned long getArmMessageCount() const { return arm_msg_count_; }
    unsigned long getJointStateCount() const { return joint_state_count_; }

private:
    // ROS objects
    rcl_node_t node_;
    rclc_executor_t executor_;
    rclc_support_t support_;
    rcl_allocator_t allocator_;
    
    // Subscribers
    rcl_subscription_t base_cmd_sub_;
    rcl_subscription_t arm_cmd_sub_;
    std_msgs__msg__Float64MultiArray base_cmd_msg_;
    std_msgs__msg__Float64MultiArray arm_cmd_msg_;
    
    // Publishers
    rcl_publisher_t joint_state_pub_;
    sensor_msgs__msg__JointState joint_state_msg_;
    
    // State tracking
    bool initialized_;
    unsigned long last_joint_state_time_;
    
    // Statistics
    unsigned long base_msg_count_;
    unsigned long arm_msg_count_;
    unsigned long joint_state_count_;
    
    // Initialization functions
    bool initializeWiFi();
    bool initializeMicroROS();
    bool createSubscribers();
    bool createPublishers();
    void setupJointStateMessage();
    
    // Callback functions (static for C compatibility)
    static void baseCommandCallback(const void* msg);
    static void armCommandCallback(const void* msg);
    
    // Helper functions
    String getErrorMessage(rcl_ret_t rc);
};

extern MicroROSManager g_microros;