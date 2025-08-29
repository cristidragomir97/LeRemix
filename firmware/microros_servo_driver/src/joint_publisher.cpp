#include "joint_publisher.h"
#include "microros.h"
#include "motor_controller.h"
#include "utils.h"
#include "config.h"
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rmw_microros/rmw_microros.h>

// Joint state publisher objects
rcl_publisher_t joint_state_pub;
sensor_msgs__msg__JointState joint_state_msg;

// Control timing
unsigned long last_joint_state_time = 0;
const unsigned long CONTROL_PERIOD_MS = 5;  // 200Hz = 5ms period

bool initJointPublisher(rcl_node_t* node, Adafruit_SSD1306* display) {
    // Create message memory for joint state
    micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      &joint_state_msg,
      (micro_ros_utilities_memory_conf_t) {
        .max_string_capacity = 20,  // For joint names
        .max_ros2_type_sequence_capacity = BASE_SERVO_COUNT + ARM_SERVO_COUNT,
        .max_basic_type_sequence_capacity = BASE_SERVO_COUNT + ARM_SERVO_COUNT
      });

    // Initialize joint names for joint state message
    joint_state_msg.name.data[0] = micro_ros_string_utilities_set(joint_state_msg.name.data[0], "back_motor_rotation");
    joint_state_msg.name.data[1] = micro_ros_string_utilities_set(joint_state_msg.name.data[1], "left_motor_rotation");
    joint_state_msg.name.data[2] = micro_ros_string_utilities_set(joint_state_msg.name.data[2], "right_motor_rotation");
    joint_state_msg.name.data[3] = micro_ros_string_utilities_set(joint_state_msg.name.data[3], "1");
    joint_state_msg.name.data[4] = micro_ros_string_utilities_set(joint_state_msg.name.data[4], "2");
    joint_state_msg.name.data[5] = micro_ros_string_utilities_set(joint_state_msg.name.data[5], "3");
    joint_state_msg.name.data[6] = micro_ros_string_utilities_set(joint_state_msg.name.data[6], "4");
    joint_state_msg.name.data[7] = micro_ros_string_utilities_set(joint_state_msg.name.data[7], "5");
    joint_state_msg.name.data[8] = micro_ros_string_utilities_set(joint_state_msg.name.data[8], "6");
    joint_state_msg.name.data[9] = micro_ros_string_utilities_set(joint_state_msg.name.data[9], "camera_tilt");
    joint_state_msg.name.size = BASE_SERVO_COUNT + ARM_SERVO_COUNT;

    // Initialize Joint State Publisher with best effort QoS for ros2_control compatibility
    rcl_ret_t rc = rclc_publisher_init_best_effort(
      &joint_state_pub, 
      node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      JOINT_STATE_TOPIC
    );

    if(rc != RCL_RET_OK) {
        displayMessage("Joint state publisher failed", display);
        return false;
    } else {
        displayMessage("Successfully added joint state publisher", display);
    }

    return true;
}

bool shouldPublishJointStates() {
    return (millis() - last_joint_state_time >= CONTROL_PERIOD_MS);
}

void publishJointStates() {
    // Read current positions from all servos
    joint_state_msg.position.size = BASE_SERVO_COUNT + ARM_SERVO_COUNT;
    joint_state_msg.velocity.size = BASE_SERVO_COUNT + ARM_SERVO_COUNT;
    joint_state_msg.effort.size = 0;  // Not using effort feedback
    
    // Update timestamp
    int64_t timestamp = rmw_uros_epoch_nanos();
    joint_state_msg.header.stamp.sec = timestamp / 1000000000ULL;
    joint_state_msg.header.stamp.nanosec = timestamp % 1000000000ULL;
    
    // Read base servo positions (convert to radians)
    for (int i = 0; i < BASE_SERVO_COUNT; i++) {
        int32_t pos = base_servo.ReadPos(BASE_SERVO_IDS[i]);
        if (pos != -1) {
            // Convert servo position to radians (for wheels, this is cumulative rotation)
            joint_state_msg.position.data[i] = (pos * 2.0 * PI) / 4096.0;
        } else {
            joint_state_msg.position.data[i] = 0.0;  // Default if read fails
        }
        
        int16_t speed = base_servo.ReadSpeed(BASE_SERVO_IDS[i]);
        if (speed != -1) {
            // Convert servo speed to rad/s
            joint_state_msg.velocity.data[i] = speed / VEL_TO_SERVO_UNIT;
        } else {
            joint_state_msg.velocity.data[i] = 0.0;
        }
    }
    
    // Read arm servo positions (convert to radians using calibration)
    for (int i = 0; i < ARM_SERVO_COUNT; i++) {
        int32_t pos = arm_servo.ReadPos(ARM_SERVO_IDS[i]);
        if (pos != -1) {
            // Use calibrated conversion function
            joint_state_msg.position.data[BASE_SERVO_COUNT + i] = servoPositionToRadians(ARM_SERVO_IDS[i], pos);
        } else {
            joint_state_msg.position.data[BASE_SERVO_COUNT + i] = 0.0;  // Default if read fails
        }
        
        int16_t speed = arm_servo.ReadSpeed(ARM_SERVO_IDS[i]);
        if (speed != -1) {
            // Convert servo speed to rad/s  
            joint_state_msg.velocity.data[BASE_SERVO_COUNT + i] = (speed * 2.0 * PI) / (4096.0 * 60.0);  // Assuming speed in RPM
        } else {
            joint_state_msg.velocity.data[BASE_SERVO_COUNT + i] = 0.0;
        }
    }
    
    // Publish joint states
    rcl_publish(&joint_state_pub, &joint_state_msg, NULL);
    last_joint_state_time = millis();
}