#include "microros_manager.h"
#include "debug_utils.h"
#include "system_state.h"
#include "servo_manager.h"

MicroRosManager g_microros;

rcl_allocator_t allocator_;
rclc_support_t support_;
rcl_node_t node_;
rcl_timer_t timer_;
rclc_executor_t executor_;

rcl_subscription_t base_cmd_sub_;
rcl_subscription_t arm_cmd_sub_;
rcl_subscription_t servo_enable_sub_;
rcl_publisher_t joint_states_pub_;

std_msgs__msg__Float64MultiArray base_cmd_msg_;
std_msgs__msg__Float64MultiArray arm_cmd_msg_;
std_msgs__msg__Bool servo_enable_msg_;
sensor_msgs__msg__JointState joint_states_msg_;

MicroRosManager::MicroRosManager() : 
    initialized_(false),
    agent_connected_(false),
    last_publish_time_(0) {}

void MicroRosManager::begin() {
    DEBUG_INFO("Initializing micro-ROS manager");
    
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
    
    delay(2000);
    
    if (initializeMicroRos()) {
        initialized_ = true;
        DEBUG_INFO("Micro-ROS manager initialized successfully");
        g_system_state.setRosConnected(true);
    } else {
        DEBUG_ERROR("Micro-ROS initialization failed");
    }
}

void MicroRosManager::update() {
    if (!initialized_) return;
    
    // Spin executor to handle callbacks
    rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(10));
    
    // Publish joint states at regular intervals
    unsigned long current_time = millis();
    if (current_time - last_publish_time_ >= PUBLISH_INTERVAL) {
        publishJointStates();
        last_publish_time_ = current_time;
    }
    
    // Check agent connection status
    checkAgentConnection();
}

bool MicroRosManager::initializeMicroRos() {
    allocator_ = rcl_get_default_allocator();
    
    // Initialize support
    if (rclc_support_init(&support_, 0, NULL, &allocator_) != RCL_RET_OK) {
        DEBUG_ERROR("Failed to initialize micro-ROS support");
        return false;
    }
    
    // Create node
    if (rclc_node_init_default(&node_, NODE_NAME, "", &support_) != RCL_RET_OK) {
        DEBUG_ERROR("Failed to create micro-ROS node");
        return false;
    }
    
    // Initialize subscriptions with best_effort QoS
    if (!initializeSubscriptions()) {
        return false;
    }
    
    // Initialize publishers with best_effort QoS
    if (!initializePublishers()) {
        return false;
    }
    
    // Create executor
    const size_t num_handles = 3; // 2 subscriptions + 1 timer
    if (rclc_executor_init(&executor_, &support_.context, num_handles, &allocator_) != RCL_RET_OK) {
        DEBUG_ERROR("Failed to create executor");
        return false;
    }
    
    // Add subscriptions to executor
    if (rclc_executor_add_subscription(&executor_, &base_cmd_sub_, &base_cmd_msg_, 
                                      &baseCmdCallback, ON_NEW_DATA) != RCL_RET_OK) {
        DEBUG_ERROR("Failed to add base command subscription");
        return false;
    }
    
    if (rclc_executor_add_subscription(&executor_, &arm_cmd_sub_, &arm_cmd_msg_,
                                      &armCmdCallback, ON_NEW_DATA) != RCL_RET_OK) {
        DEBUG_ERROR("Failed to add arm command subscription");
        return false;
    }
    
    if (rclc_executor_add_subscription(&executor_, &servo_enable_sub_, &servo_enable_msg_,
                                      &servoEnableCallback, ON_NEW_DATA) != RCL_RET_OK) {
        DEBUG_ERROR("Failed to add servo enable subscription");
        return false;
    }
    
    DEBUG_INFO("Micro-ROS initialization successful");
    return true;
}

bool MicroRosManager::initializeSubscriptions() {
    // Initialize message arrays
    base_cmd_msg_.data.capacity = BASE_SERVO_COUNT;
    base_cmd_msg_.data.size = 0;
    base_cmd_msg_.data.data = (double*)malloc(base_cmd_msg_.data.capacity * sizeof(double));
    if (base_cmd_msg_.data.data == NULL) {
        DEBUG_ERROR("Failed to allocate base command message");
        return false;
    }
    
    arm_cmd_msg_.data.capacity = ARM_SERVO_COUNT;
    arm_cmd_msg_.data.size = 0;
    arm_cmd_msg_.data.data = (double*)malloc(arm_cmd_msg_.data.capacity * sizeof(double));
    if (arm_cmd_msg_.data.data == NULL) {
        DEBUG_ERROR("Failed to allocate arm command message");
        return false;
    }
    
    // Create subscriptions with best_effort QoS
    rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
    subscription_ops.qos.reliability = RCL_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    
    if (rclc_subscription_init_best_effort(&base_cmd_sub_, &node_, 
                                          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
                                          BASE_CMD_TOPIC) != RCL_RET_OK) {
        DEBUG_ERROR("Failed to create base command subscription");
        return false;
    }
    
    if (rclc_subscription_init_best_effort(&arm_cmd_sub_, &node_,
                                          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
                                          ARM_CMD_TOPIC) != RCL_RET_OK) {
        DEBUG_ERROR("Failed to create arm command subscription");
        return false;
    }
    
    if (rclc_subscription_init_best_effort(&servo_enable_sub_, &node_,
                                          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                          SERVO_ENABLE_TOPIC) != RCL_RET_OK) {
        DEBUG_ERROR("Failed to create servo enable subscription");
        return false;
    }
    
    DEBUG_INFO("Subscriptions initialized");
    return true;
}

bool MicroRosManager::initializePublishers() {
    // Initialize joint state message
    joint_states_msg_.name.capacity = TOTAL_SERVO_COUNT;
    joint_states_msg_.name.size = TOTAL_SERVO_COUNT;
    joint_states_msg_.name.data = (rosidl_runtime_c__String*)malloc(joint_states_msg_.name.capacity * sizeof(rosidl_runtime_c__String));
    
    joint_states_msg_.position.capacity = TOTAL_SERVO_COUNT;
    joint_states_msg_.position.size = TOTAL_SERVO_COUNT;
    joint_states_msg_.position.data = (double*)malloc(joint_states_msg_.position.capacity * sizeof(double));
    
    joint_states_msg_.velocity.capacity = TOTAL_SERVO_COUNT;
    joint_states_msg_.velocity.size = TOTAL_SERVO_COUNT;
    joint_states_msg_.velocity.data = (double*)malloc(joint_states_msg_.velocity.capacity * sizeof(double));
    
    if (joint_states_msg_.name.data == NULL || 
        joint_states_msg_.position.data == NULL || 
        joint_states_msg_.velocity.data == NULL) {
        DEBUG_ERROR("Failed to allocate joint states message");
        return false;
    }
    
    // Initialize joint names
    const char* joint_names[] = {
        "base_left_wheel", "base_right_wheel",
        "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4"
    };
    
    for (size_t i = 0; i < TOTAL_SERVO_COUNT; i++) {
        rosidl_runtime_c__String__init(&joint_states_msg_.name.data[i]);
        rosidl_runtime_c__String__assign(&joint_states_msg_.name.data[i], joint_names[i]);
    }
    
    // Create publisher with best_effort QoS
    if (rclc_publisher_init_best_effort(&joint_states_pub_, &node_,
                                       ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                                       JOINT_STATES_TOPIC) != RCL_RET_OK) {
        DEBUG_ERROR("Failed to create joint states publisher");
        return false;
    }
    
    DEBUG_INFO("Publishers initialized");
    return true;
}

void MicroRosManager::publishJointStates() {
    if (!initialized_) return;
    
    // Read current joint states from servo manager
    float positions[TOTAL_SERVO_COUNT];
    float velocities[TOTAL_SERVO_COUNT];
    
    if (g_servo.readJointStates(positions, velocities, TOTAL_SERVO_COUNT)) {
        // Update message
        for (size_t i = 0; i < TOTAL_SERVO_COUNT; i++) {
            joint_states_msg_.position.data[i] = positions[i];
            joint_states_msg_.velocity.data[i] = velocities[i];
        }
        
        // Set timestamp
        int64_t time_ns = rmw_uros_epoch_nanos();
        joint_states_msg_.header.stamp.sec = time_ns / 1000000000;
        joint_states_msg_.header.stamp.nanosec = time_ns % 1000000000;
        
        // Set frame_id
        rosidl_runtime_c__String__assign(&joint_states_msg_.header.frame_id, "base_link");
        
        // Publish
        rcl_ret_t ret = rcl_publish(&joint_states_pub_, &joint_states_msg_, NULL);
        if (ret == RCL_RET_OK) {
            DEBUG_PRINTF("Published joint states (rc=%d)", ret);
        } else {
            DEBUG_PRINTF("Failed to publish joint states (rc=%d)", ret);
        }
    }
}

void MicroRosManager::checkAgentConnection() {
    bool was_connected = agent_connected_;
    agent_connected_ = rmw_uros_ping_agent(100, 1) == RCL_RET_OK;
    
    if (agent_connected_ != was_connected) {
        if (agent_connected_) {
            DEBUG_INFO("Micro-ROS agent connection established");
            g_system_state.setRosConnected(true);
            g_system_state.enableWatchdog();
        } else {
            DEBUG_WARNING("Micro-ROS agent connection lost");
            g_system_state.setRosConnected(false);
        }
    }
}

void baseCmdCallback(const void* msgin) {
    const std_msgs__msg__Float64MultiArray* msg = (const std_msgs__msg__Float64MultiArray*)msgin;
    
    DEBUG_PRINTF("Base command received: size=%d, capacity=%d", 
                 msg->data.size, msg->data.capacity);
    
    if (msg->data.size > 0 && msg->data.data != NULL) {
        // Convert double array to float array
        float velocities[BASE_SERVO_COUNT];
        for (size_t i = 0; i < msg->data.size && i < BASE_SERVO_COUNT; i++) {
            velocities[i] = (float)msg->data.data[i];
            DEBUG_PRINTF("Base vel[%d] = %.3f", i, velocities[i]);
        }
        
        // Process command
        g_servo.processBaseCommand(velocities, msg->data.size);
        g_system_state.updateBaseCommandTime();
        
    } else {
        DEBUG_WARNING("Base command: NULL data received");
    }
}

void armCmdCallback(const void* msgin) {
    const std_msgs__msg__Float64MultiArray* msg = (const std_msgs__msg__Float64MultiArray*)msgin;
    
    DEBUG_PRINTF("Arm command received: size=%d, capacity=%d", 
                 msg->data.size, msg->data.capacity);
    
    if (msg->data.size > 0 && msg->data.data != NULL) {
        // Convert double array to float array
        float positions[ARM_SERVO_COUNT];
        for (size_t i = 0; i < msg->data.size && i < ARM_SERVO_COUNT; i++) {
            positions[i] = (float)msg->data.data[i];
            DEBUG_PRINTF("Arm pos[%d] = %.3f", i, positions[i]);
        }
        
        // Process command
        g_servo.processArmCommand(positions, msg->data.size);
        g_system_state.updateArmCommandTime();
        
    } else {
        DEBUG_WARNING("Arm command: NULL data received");
    }
}

void servoEnableCallback(const void* msgin) {
    const std_msgs__msg__Bool* msg = (const std_msgs__msg__Bool*)msgin;
    
    DEBUG_PRINTF("Servo enable command: %s", msg->data ? "ENABLE" : "DISABLE");
    
    g_system_state.setServosEnabled(msg->data);
    
    if (msg->data) {
        g_servo.enableAllServos();
    } else {
        g_servo.disableAllServos();
    }
}