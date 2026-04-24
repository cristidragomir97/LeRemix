#include "llmy_control_plugin/ros2_control_bridge.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <limits>
#include <chrono>
#include <thread>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

namespace llmy_control_plugin {

CallbackReturn ROS2ControlBridge::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  // Params from ros2_control yaml
  auto it = info_.hardware_parameters.find("base_cmd_topic");
  if (it != info_.hardware_parameters.end()) base_cmd_topic_ = it->second;
  it = info_.hardware_parameters.find("arm_cmd_topic");
  if (it != info_.hardware_parameters.end()) arm_cmd_topic_ = it->second;
  it = info_.hardware_parameters.find("camera_cmd_topic");
  if (it != info_.hardware_parameters.end()) camera_cmd_topic_ = it->second;
  it = info_.hardware_parameters.find("state_topic");
  if (it != info_.hardware_parameters.end()) state_topic_ = it->second;
  it = info_.hardware_parameters.find("publish_if_unchanged");
  if (it != info_.hardware_parameters.end()) publish_if_unchanged_ = (it->second == "true" || it->second == "1");

  // Split joints into base (velocity cmd), arm (position cmd), and camera (position cmd)
  for (const auto & j : info_.joints) {
    bool has_vel_cmd = false, has_pos_cmd = false;
    for (const auto & ci : j.command_interfaces) {
      if (ci.name == HW_IF_VELOCITY) has_vel_cmd = true;
      if (ci.name == HW_IF_POSITION) has_pos_cmd = true;
    }
    if (has_vel_cmd) {
      base_joints_.push_back(j.name);
      cmd_vel_[j.name] = 0.0;
      pos_state_[j.name] = 0.0;
      vel_state_[j.name] = 0.0;
    }
    if (has_pos_cmd) {
      // Classify position joints as arm or camera based on joint name
      if (j.name.find("camera") != std::string::npos) {
        camera_joints_.push_back(j.name);
      } else {
        arm_joints_.push_back(j.name);
      }
      cmd_pos_[j.name] = std::numeric_limits<double>::quiet_NaN();  // Use NaN to indicate no command received
      cmd_pos_received_[j.name] = false;  // Track that no command has been received yet
      pos_state_[j.name] = 0.0;
      vel_state_[j.name] = 0.0;  // velocity state for trajectory controller feedback
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ROS2ControlBridge::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // Base joints: position & velocity
  for (const auto & name : base_joints_) {
    state_interfaces.emplace_back(name, HW_IF_POSITION, &pos_state_[name]);
    state_interfaces.emplace_back(name, HW_IF_VELOCITY, &vel_state_[name]);
  }
  // Arm joints: position & velocity
  for (const auto & name : arm_joints_) {
    state_interfaces.emplace_back(name, HW_IF_POSITION, &pos_state_[name]);
    state_interfaces.emplace_back(name, HW_IF_VELOCITY, &vel_state_[name]);
  }
  // Camera joints: position
  for (const auto & name : camera_joints_) {
    state_interfaces.emplace_back(name, HW_IF_POSITION, &pos_state_[name]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ROS2ControlBridge::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const auto & name : base_joints_) {
    command_interfaces.emplace_back(name, HW_IF_VELOCITY, &cmd_vel_[name]);
  }
  for (const auto & name : arm_joints_) {
    command_interfaces.emplace_back(name, HW_IF_POSITION, &cmd_pos_[name]);
  }
  for (const auto & name : camera_joints_) {
    command_interfaces.emplace_back(name, HW_IF_POSITION, &cmd_pos_[name]);
  }
  return command_interfaces;
}

CallbackReturn ROS2ControlBridge::on_configure(const rclcpp_lifecycle::State &)
{
  // Create our internal node and pubs/subs
  node_ = std::make_shared<rclcpp::Node>("ROSControlMotorBridge");

  // Publishers (best-effort for micro-ROS compatibility)
  base_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    base_cmd_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
  arm_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    arm_cmd_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
  camera_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    camera_cmd_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

  // Subscriber (best-effort for micro-ROS compatibility)
  auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
  state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    state_topic_, qos,
    std::bind(&ROS2ControlBridge::state_callback, this, std::placeholders::_1));

  exec_.add_node(node_);

  RCLCPP_INFO(node_->get_logger(), "Configured MotorBridge. base_cmd_topic=%s arm_cmd_topic=%s camera_cmd_topic=%s state_topic=%s",
              base_cmd_topic_.c_str(), arm_cmd_topic_.c_str(), camera_cmd_topic_.c_str(), state_topic_.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn ROS2ControlBridge::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(node_->get_logger(), "ROS2 Control Bridge activated - will wait for meaningful joint states before accepting commands");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ROS2ControlBridge::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Shutdown node to stop callbacks
  exec_.remove_node(node_);
  state_sub_.reset();
  base_pub_.reset();
  arm_pub_.reset();
  camera_pub_.reset();
  node_.reset();
  return CallbackReturn::SUCCESS;
}

void ROS2ControlBridge::state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::scoped_lock<std::mutex> lk(state_mtx_);

  const size_t n = msg->name.size();
  bool has_meaningful_position = false;

  for (size_t i = 0; i < n; ++i) {
    const auto & name = msg->name[i];

    if (pos_state_.count(name)) {
      if (i < msg->position.size()) {
        pos_state_[name] = msg->position[i];
        // Check if this is a meaningful (non-zero) position for arm/camera joints
        if ((std::find(arm_joints_.begin(), arm_joints_.end(), name) != arm_joints_.end() ||
             std::find(camera_joints_.begin(), camera_joints_.end(), name) != camera_joints_.end()) &&
            std::abs(msg->position[i]) > 0.005) {  // More than 0.005 radians (~0.3 degrees)
          has_meaningful_position = true;
        }
      }
    }
    if (vel_state_.count(name)) {
      if (i < msg->velocity.size()) vel_state_[name] = msg->velocity[i];
    }
  }

  // Mark that we've received meaningful joint states
  if (has_meaningful_position && !received_meaningful_joint_states_) {
    received_meaningful_joint_states_ = true;
    RCLCPP_INFO(node_->get_logger(), "Received meaningful joint states - ready to accept controller commands");
  }
}

return_type ROS2ControlBridge::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Service any pending JointState messages without blocking controller timings
  exec_.spin_some();
  return return_type::OK;
}

return_type ROS2ControlBridge::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Check if we've received meaningful joint states yet
  if (!received_meaningful_joint_states_) {
    // Until we get meaningful joint states, override any controller commands with current positions
    {
      std::scoped_lock<std::mutex> lk(state_mtx_);
      for (const auto & j : arm_joints_) {
        if (pos_state_.count(j)) {
          cmd_pos_[j] = pos_state_[j];  // Keep current position
        }
      }
      for (const auto & j : camera_joints_) {
        if (pos_state_.count(j)) {
          cmd_pos_[j] = pos_state_[j];  // Keep current position
        }
      }
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Waiting for meaningful joint states - holding current positions");
  } else {
    // After receiving meaningful joint states, process commands normally
    for (const auto & j : arm_joints_) {
      if (!cmd_pos_received_[j]) {
        cmd_pos_received_[j] = true;
      }
    }
    for (const auto & j : camera_joints_) {
      if (!cmd_pos_received_[j]) {
        cmd_pos_received_[j] = true;
      }
    }
  }

  // Publish base velocities in the order of base_joints_ (always publish - locomotion not affected by joint state waiting)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(base_joints_.size());
    for (const auto & j : base_joints_) {
      msg.data.push_back(cmd_vel_[j]);
    }
    base_pub_->publish(msg);
  }

  // Publish arm positions in the order of arm_joints_ - only if valid commands received
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(arm_joints_.size());
    bool has_valid_cmd = false;
    for (const auto & j : arm_joints_) {
      if (cmd_pos_received_[j] && !std::isnan(cmd_pos_[j])) {
        msg.data.push_back(cmd_pos_[j]);
        has_valid_cmd = true;
      } else {
        msg.data.push_back(0.0);  // Placeholder, but message won't be published without valid commands
      }
    }
    if (has_valid_cmd) {
      arm_pub_->publish(msg);
    }
  }

  // Publish camera positions in the order of camera_joints_ - only if valid commands received
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(camera_joints_.size());
    bool has_valid_cmd = false;
    for (const auto & j : camera_joints_) {
      if (cmd_pos_received_[j] && !std::isnan(cmd_pos_[j])) {
        msg.data.push_back(cmd_pos_[j]);
        has_valid_cmd = true;
      } else {
        msg.data.push_back(0.0);  // Placeholder, but message won't be published without valid commands
      }
    }
    if (has_valid_cmd) {
      camera_pub_->publish(msg);
    }
  }

  return return_type::OK;
}

} // namespace llmy_control_plugin

PLUGINLIB_EXPORT_CLASS(llmy_control_plugin::ROS2ControlBridge, hardware_interface::SystemInterface)
