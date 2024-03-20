#pragma once

#include <lcm/lcm-cpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <string>
#include <victor_hardware/lcm_listener.hpp>
#include <victor_hardware_interfaces/msg/motion_command.hpp>
#include <victor_hardware_interfaces/msg/motion_status.hpp>
#include <victor_hardware_interfaces/msg/control_mode_parameters.hpp>
#include <victor_hardware_interfaces/msg/robotiq3_finger_command.hpp>
#include <victor_hardware_interfaces/msg/robotiq3_finger_status.hpp>
#include <victor_hardware_interfaces/srv/set_control_mode.hpp>
#include <victor_hardware_interfaces/srv/get_control_mode.hpp>
#include <victor_lcm_interface/control_mode_parameters.hpp>
#include <victor_lcm_interface/motion_status.hpp>
#include <victor_lcm_interface/robotiq_3finger_status.hpp>

namespace msg = victor_hardware_interfaces::msg;
namespace srv = victor_hardware_interfaces::srv;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace victor_hardware {

class Side {
 public:
  explicit Side(std::string const& name);

  CallbackReturn on_init(std::shared_ptr<rclcpp::Node> const& node, std::string const& send_provider, std::string const& recv_provider);

  void publish_motion_status(victor_lcm_interface::motion_status const &msg);
  void publish_gripper_status(victor_lcm_interface::robotiq_3finger_status const &msg);

  std::string name_;
  std::array<double, 6> hw_ft_;
  geometry_msgs::msg::Pose hw_state_cartesian_pose_;
  geometry_msgs::msg::Pose hw_state_cmd_cartesian_pose_;

  std::shared_ptr<lcm::LCM> send_lcm_ptr_;
  std::shared_ptr<lcm::LCM> recv_lcm_ptr_;

  std::string cartesian_control_frame_;

  std::unique_ptr<LcmListener<victor_lcm_interface::motion_status>> motion_status_listener_;
  std::unique_ptr<LcmListener<victor_lcm_interface::control_mode_parameters>> control_mode_listener_;
  std::unique_ptr<LcmListener<victor_lcm_interface::robotiq_3finger_status>> gripper_status_listener_;

  // ROS API
  std::shared_ptr<rclcpp::Publisher<msg::MotionStatus>> motion_status_pub_;
  std::shared_ptr<rclcpp::Publisher<msg::Robotiq3FingerStatus>> gripper_status_pub_;


 private:
  rclcpp::Logger logger_;
};

}  // namespace victor_hardware
