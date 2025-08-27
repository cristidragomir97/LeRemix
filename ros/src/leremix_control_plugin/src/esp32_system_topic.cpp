#include "leremix_control_plugin/esp32_system_topic.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <algorithm>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

namespace leremix_control_plugin {

CallbackReturn Esp32SystemTopic::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  // Params from ros2_control yaml
  auto it = info_.hardware_parameters.find("base_cmd_topic");
  if (it != info_.hardware_parameters.end()) base_cmd_topic_ = it->second;
  it = info_.hardware_parameters.find("arm_cmd_topic");
  if (it != info_.hardware_parameters.end()) arm_cmd_topic_ = it->second;
  it = info_.hardware_parameters.find("state_topic");
  if (it != info_.hardware_parameters.end()) state_topic_ = it->second;
  it = info_.hardware_parameters.find("publish_if_unchanged");
  if (it != info_.hardware_parameters.end()) publish_if_unchanged_ = (it->second == "true" || it->second == "1");

  // Split joints into base (velocity cmd) and arm (position cmd)
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
      arm_joints_.push_back(j.name);
      cmd_pos_[j.name] = 0.0;
      pos_state_[j.name] = 0.0;
      // vel_state_ optional for arm; leave default 0
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Esp32SystemTopic::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // Base joints: position & velocity
  for (const auto & name : base_joints_) {
    state_interfaces.emplace_back(name, HW_IF_POSITION, &pos_state_[name]);
    state_interfaces.emplace_back(name, HW_IF_VELOCITY, &vel_state_[name]);
  }
  // Arm joints: position
  for (const auto & name : arm_joints_) {
    state_interfaces.emplace_back(name, HW_IF_POSITION, &pos_state_[name]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Esp32SystemTopic::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const auto & name : base_joints_) {
    command_interfaces.emplace_back(name, HW_IF_VELOCITY, &cmd_vel_[name]);
  }
  for (const auto & name : arm_joints_) {
    command_interfaces.emplace_back(name, HW_IF_POSITION, &cmd_pos_[name]);
  }
  return command_interfaces;
}

CallbackReturn Esp32SystemTopic::on_configure(const rclcpp_lifecycle::State &)
{
  // Create our internal node and pubs/subs
  node_ = std::make_shared<rclcpp::Node>("esp32_hw_bridge");

  // Publishers (best-effort, depth 1)
  base_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    base_cmd_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
  arm_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    arm_cmd_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

  // Subscriber (reliable, depth 5 – adjust to BEST_EFFORT if needed)
  auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
  state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    state_topic_, qos,
    std::bind(&Esp32SystemTopic::state_callback, this, std::placeholders::_1));

  exec_.add_node(node_);

  RCLCPP_INFO(node_->get_logger(), "Configured Esp32SystemTopic. base_cmd_topic=%s arm_cmd_topic=%s state_topic=%s",
              base_cmd_topic_.c_str(), arm_cmd_topic_.c_str(), state_topic_.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn Esp32SystemTopic::on_activate(const rclcpp_lifecycle::State &)
{
  // Nothing special, topics already created
  return CallbackReturn::SUCCESS;
}

CallbackReturn Esp32SystemTopic::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Shutdown node to stop callbacks
  exec_.remove_node(node_);
  state_sub_.reset();
  base_pub_.reset();
  arm_pub_.reset();
  node_.reset();
  return CallbackReturn::SUCCESS;
}

void Esp32SystemTopic::state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::scoped_lock<std::mutex> lk(state_mtx_);

  const size_t n = msg->name.size();
  for (size_t i = 0; i < n; ++i) {
    const auto & name = msg->name[i];

    if (pos_state_.count(name)) {
      if (i < msg->position.size()) pos_state_[name] = msg->position[i];
    }
    if (vel_state_.count(name)) {
      if (i < msg->velocity.size()) vel_state_[name] = msg->velocity[i];
    }
  }
}

return_type Esp32SystemTopic::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Service any pending JointState messages without blocking controller timings
  exec_.spin_some();
  return return_type::OK;
}

return_type Esp32SystemTopic::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Publish base velocities in the order of base_joints_
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(base_joints_.size());
    for (const auto & j : base_joints_) {
      msg.data.push_back(cmd_vel_[j]);
    }
    base_pub_->publish(msg);
  }

  // Publish arm positions in the order of arm_joints_
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(arm_joints_.size());
    for (const auto & j : arm_joints_) {
      msg.data.push_back(cmd_pos_[j]);
    }
    arm_pub_->publish(msg);
  }

  return return_type::OK;
}

} // namespace leremix_control_plugin

PLUGINLIB_EXPORT_CLASS(leremix_control_plugin::Esp32SystemTopic, hardware_interface::SystemInterface)
