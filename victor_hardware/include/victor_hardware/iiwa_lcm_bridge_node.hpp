#pragma once

#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>

// ROS message headers
#include <victor_hardware_interfaces/msg/control_mode_parameters.hpp>
#include <victor_hardware_interfaces/msg/motion_command.hpp>
#include <victor_hardware_interfaces/msg/motion_status.hpp>
#include <victor_hardware_interfaces/msg/robotiq3_finger_command.hpp>
#include <victor_hardware_interfaces/msg/robotiq3_finger_status.hpp>
#include <victor_hardware_interfaces/srv/get_control_mode.hpp>
#include <victor_hardware_interfaces/srv/set_control_mode.hpp>

// LCM
#include <lcm/lcm-cpp.hpp>

// Classes to speak to each individual hardware element
#include <victor_hardware/constants.hpp>
#include <victor_hardware/iiwa_lcm_bridge.hpp>
#include <victor_hardware/robotiq_3f_lcm_bridge.hpp>
#include <victor_hardware/victor_lcm_bridge_utils.hpp>

using namespace victor_hardware;
namespace msg = victor_hardware_interfaces::msg;
namespace srv = victor_hardware_interfaces::srv;

class IiwaLcmBridgeNode : public rclcpp::Node {
 public:
  IiwaLcmBridgeNode(std::string const& node_name, std::shared_ptr<lcm::LCM> send_lcm_ptr, std::shared_ptr<lcm::LCM> recv_lcm_ptr);

  static std::pair<bool, std::string> validateJointPathExecutionParams(const msg::JointPathExecutionParameters& params);

  std::pair<bool, std::string> validateCartesianPathExecutionParams(const msg::CartesianPathExecutionParameters& params);

  std::pair<bool, std::string> validateJointImpedanceParams(const msg::JointImpedanceParameters& params);

  std::pair<bool, std::string> validateCartesianImpedanceParams(const msg::CartesianImpedanceParameters& params);

  std::pair<bool, std::string> validateCartesianControlModeLimits(const msg::CartesianControlModeLimits& params);

  std::pair<bool, std::string> validateControlMode(const msg::ControlModeParameters& params);
  void setControlModeCallback(const srv::SetControlMode::Request::SharedPtr req,
                              srv::SetControlMode::Response::SharedPtr res);

  void getControlModeCallback(const srv::GetControlMode::Request::SharedPtr /*req*/,
                              srv::GetControlMode::Response::SharedPtr res);

  void controlModeStatusLCMCallback(const msg::ControlModeParameters& control_mode_status);
  std::pair<bool, std::string> validateCartesianPose(const geometry_msgs::msg::Pose& pose,
                                                     const std::string& frame) const;

  std::pair<bool, std::string> validateMotionCommand(const msg::MotionCommand& command) const;

  void motionCommandROSCallback(const msg::MotionCommand& command);

  void motionStatusLCMCallback(const msg::MotionStatus& motion_status);

  static std::pair<bool, std::string> validateFingerCommand(const msg::Robotiq3FingerActuatorCommand& command);

  std::pair<bool, std::string> validateGripperCommand(const msg::Robotiq3FingerCommand& command);

  void gripperCommandROSCallback(const msg::Robotiq3FingerCommand& command);

  void gripperStatusLCMCallback(const msg::Robotiq3FingerStatus& gripper_status);

  static bool jointPathExecutionParamsIsDefault(const msg::JointPathExecutionParameters& params);

  static bool cartesianPathExecutionParamsIsDefault(const msg::CartesianPathExecutionParameters& params);

  static bool jointImpedanceParamsIsDefault(const msg::JointImpedanceParameters& params);

  static bool cartesianImpedanceParamsIsDefault(const msg::CartesianImpedanceParameters& params);

  static bool cartesianControlModeLimitsIsDefault(const msg::CartesianControlModeLimits& params);

  static msg::ControlModeParameters mergeControlModeParameters(
      const msg::ControlModeParameters& active_control_mode, const msg::ControlModeParameters& new_control_mode);

 private:

  std::string cartesian_control_frame_;
  rclcpp::Publisher<msg::MotionStatus>::SharedPtr motion_status_pub_;
  rclcpp::Publisher<msg::ControlModeParameters>::SharedPtr control_mode_status_pub_;
  rclcpp::Publisher<msg::Robotiq3FingerStatus>::SharedPtr gripper_status_pub_;
  rclcpp::Subscription<msg::MotionCommand>::SharedPtr motion_command_sub_;
  rclcpp::Subscription<msg::Robotiq3FingerCommand>::SharedPtr gripper_command_sub_;
  rclcpp::Service<srv::SetControlMode>::SharedPtr set_control_mode_server_;
  rclcpp::Service<srv::GetControlMode>::SharedPtr get_control_mode_server_;

  // callback groups for each ROS thing
  rclcpp::CallbackGroup::SharedPtr getter_callback_group_;
  rclcpp::CallbackGroup::SharedPtr setter_callback_group_;

  mutable std::mutex control_mode_status_mutex_;
  std::optional<msg::ControlModeParameters> active_control_mode_;
  double set_control_mode_timeout_;

  std::shared_ptr<lcm::LCM> send_lcm_ptr_;
  std::shared_ptr<lcm::LCM> recv_lcm_ptr_;
  std::unique_ptr<IiwaLcmBridge> iiwa_ptr_;
  std::unique_ptr<Robotiq3fLcmBridge> robotiq_ptr_;
};
