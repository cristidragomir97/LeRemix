#pragma once
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>

namespace llmy_control_plugin {

class ROS2ControlBridge : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ROS2ControlBridge)

  // Lifecycle
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // I/O
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters
  std::string base_cmd_topic_{"/motor_manager/base_cmd"};
  std::string arm_cmd_topic_{"/motor_manager/arm_cmd"};
  std::string camera_cmd_topic_{"/motor_manager/camera_cmd"};
  std::string state_topic_{"/motor_manager/joint_states"};
  bool publish_if_unchanged_{true};

  // Joint sets derived from interfaces:
  std::vector<std::string> base_joints_;   // velocity command
  std::vector<std::string> arm_joints_;    // position command
  std::vector<std::string> camera_joints_; // position command

  // Buffers
  std::unordered_map<std::string, double> cmd_vel_;   // for base joints (rad/s)
  std::unordered_map<std::string, double> cmd_pos_;   // for arm+camera joints (rad)

  std::unordered_map<std::string, double> pos_state_; // all joints (rad)
  std::unordered_map<std::string, double> vel_state_; // base joints (rad/s), arm optional

  // Track whether valid commands have been received to prevent initial 0.0 commands
  std::unordered_map<std::string, bool> cmd_pos_received_;  // for arm+camera joints

  // Track if we've received meaningful (non-zero) joint states
  bool received_meaningful_joint_states_{false};

  // ROS
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr base_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr camera_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  std::mutex state_mtx_;

  // Helpers
  void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

} // namespace llmy_control_plugin
