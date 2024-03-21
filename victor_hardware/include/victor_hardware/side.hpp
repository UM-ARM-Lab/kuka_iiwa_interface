#pragma once
#pragma once

#include <lcm/lcm-cpp.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <string>
#include <unordered_map>
#include <victor_hardware/lcm_listener.hpp>
#include <victor_hardware_interfaces/msg/control_mode_parameters.hpp>
#include <victor_hardware_interfaces/msg/motion_command.hpp>
#include <victor_hardware_interfaces/msg/motion_status.hpp>
#include <victor_hardware_interfaces/msg/robotiq3_finger_command.hpp>
#include <victor_hardware_interfaces/msg/robotiq3_finger_status.hpp>
#include <victor_hardware_interfaces/srv/get_control_mode.hpp>
#include <victor_hardware_interfaces/srv/set_control_mode.hpp>
#include <victor_lcm_interface/control_mode_parameters.hpp>
#include <victor_lcm_interface/motion_command.hpp>
#include <victor_lcm_interface/motion_status.hpp>
#include <victor_lcm_interface/robotiq_3finger_status.hpp>

namespace msg = victor_hardware_interfaces::msg;
namespace srv = victor_hardware_interfaces::srv;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace victor_hardware {

class Side {
 public:
  explicit Side(std::string const& name);

  CallbackReturn on_init(std::shared_ptr<rclcpp::Executor> const& executor, std::shared_ptr<rclcpp::Node> const& node,
                         std::string const& send_provider, std::string const& recv_provider);

  void getControlMode(const std::shared_ptr<srv::GetControlMode::Request>& request,
                      std::shared_ptr<srv::GetControlMode::Response> response) const;
  void setControlMode(const std::shared_ptr<srv::SetControlMode::Request>& request,
                      std::shared_ptr<srv::SetControlMode::Response> response);
  void gripperCommandROSCallback(const msg::Robotiq3FingerCommand& command);
  void publish_motion_status(victor_lcm_interface::motion_status const& msg);
  void publish_gripper_status(victor_lcm_interface::robotiq_3finger_status const& msg);
  void send_motion_command(victor_lcm_interface::motion_command const& msg);

  std::string name_;
  bool send_motion_command_{true};

  // These get bound to command interfaces
  geometry_msgs::msg::Pose hw_cmd_cartesian_pose_;
  // TODO: should joint positions go here too? Should everything that's bound to state or command get moved
  //  from the hw if to here?

  // These get bound to state interfaces
  std::array<double, 6> hw_ft_;
  geometry_msgs::msg::Pose hw_state_cartesian_pose_;
  geometry_msgs::msg::Pose hw_state_cmd_cartesian_pose_;

  std::shared_ptr<lcm::LCM> send_lcm_ptr_;
  std::shared_ptr<lcm::LCM> recv_lcm_ptr_;

  std::string cartesian_control_frame_;

  std::unique_ptr<LcmListener<victor_lcm_interface::motion_status>> motion_status_listener_;
  std::unique_ptr<LcmListener<victor_lcm_interface::control_mode_parameters>> control_mode_listener_;
  std::unique_ptr<LcmListener<victor_lcm_interface::robotiq_3finger_status>> gripper_status_listener_;
  rclcpp::Service<srv::SetControlMode>::SharedPtr set_control_mode_server_;
  rclcpp::Service<srv::GetControlMode>::SharedPtr get_control_mode_server_;

  // ROS API
  rclcpp::Publisher<msg::MotionStatus>::SharedPtr motion_status_pub_;
  rclcpp::Publisher<msg::Robotiq3FingerStatus>::SharedPtr gripper_status_pub_;
  rclcpp::Subscription<msg::Robotiq3FingerCommand>::SharedPtr gripper_command_sub_;

  // callback groups for each ROS thing
  rclcpp::CallbackGroup::SharedPtr getter_callback_group_;
  rclcpp::CallbackGroup::SharedPtr setter_callback_group_;

 private:
  rclcpp::Logger logger_;
  std::unordered_map<std::string, std::vector<uint8_t>> valid_modes_for_controllers_map_;

  std::pair<bool, std::string> validateControlModeRequest(
      const victor_hardware_interfaces::srv::SetControlMode::Request& request);
};

}  // namespace victor_hardware
