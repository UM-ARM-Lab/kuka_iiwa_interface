#pragma once

#include <lcm/lcm-cpp.hpp>
#include <string>
// ROS message headers
#include <geometry_msgs/msg/pose.hpp>
#include "victor_hardware_interfaces/msg/control_mode_parameters.hpp"
#include "victor_hardware_interfaces/msg/motion_command.hpp"
#include "victor_hardware_interfaces/msg/motion_status.hpp"
// LCM type headers
#include "victor_lcm_interface/control_mode_parameters.hpp"
#include "victor_lcm_interface/motion_command.hpp"
#include "victor_lcm_interface/motion_status.hpp"

namespace victor_hardware {
namespace msg = victor_hardware_interfaces::msg;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LCM to ROS and ROS to LCM message converters
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

msg::JointValueQuantity jvqLcmToRos(const victor_lcm_interface::joint_value_quantity& lcm_jvq);
victor_lcm_interface::joint_value_quantity jvqRosToLcm(const msg::JointValueQuantity& ros_jvq);

msg::CartesianValueQuantity cvqLcmToRos(const victor_lcm_interface::cartesian_value_quantity& lcm_cvq);
victor_lcm_interface::cartesian_value_quantity cvqRosToLcm(const msg::CartesianValueQuantity& ros_cvq);

geometry_msgs::msg::Pose poseLcmToRos(const victor_lcm_interface::cartesian_pose& lcm_pose);
victor_lcm_interface::cartesian_pose poseRosToLcm(const geometry_msgs::msg::Pose& ros_pose);

msg::JointImpedanceParameters jointImpedanceParamsLcmToRos(const victor_lcm_interface::joint_impedance_parameters& lcm_jip);
victor_lcm_interface::joint_impedance_parameters jointImpedanceParamsRosToLcm(const msg::JointImpedanceParameters& ros_jip);

msg::CartesianImpedanceParameters cartesianImpedanceParamsLcmToRos(const victor_lcm_interface::cartesian_impedance_parameters& lcm_cip);
victor_lcm_interface::cartesian_impedance_parameters cartesianImpedanceParamsRosToLcm(const msg::CartesianImpedanceParameters& ros_cip);

msg::JointPathExecutionParameters jointPexpLcmToRos(const victor_lcm_interface::joint_path_execution_parameters& path_execution_params);
victor_lcm_interface::joint_path_execution_parameters jointPexpRosToLcm(const msg::JointPathExecutionParameters& path_execution_params);

msg::CartesianPathExecutionParameters cartesianPexpLcmToRos(const victor_lcm_interface::cartesian_path_execution_parameters& lcm_pexp);
victor_lcm_interface::cartesian_path_execution_parameters cartesianPexpRosToLcm(const msg::CartesianPathExecutionParameters& ros_pexp);

msg::CartesianControlModeLimits cartesianControlModeLimitsLcmToRos(const victor_lcm_interface::cartesian_control_mode_limits& lcm_ccml);
victor_lcm_interface::cartesian_control_mode_limits cartesianControlModeLimitsRosToLcm(const msg::CartesianControlModeLimits& ros_ccml);

msg::ControlMode controlModeLcmToRos(const victor_lcm_interface::control_mode& lcm_cm);
victor_lcm_interface::control_mode controlModeRosToLcm(const msg::ControlMode& ros_cm);

msg::MotionStatus motionStatusLcmToRos(const victor_lcm_interface::motion_status& lcm_status);
victor_lcm_interface::motion_command motionCommandRosToLcm(const msg::MotionCommand& ros_command);

msg::ControlModeParameters controlModeParamsLcmToRos(const victor_lcm_interface::control_mode_parameters& lcm_cmp);
victor_lcm_interface::control_mode_parameters controlModeParamsRosToLcm(const msg::ControlModeParameters& ros_cmp);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The class that does the actual communication
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class IiwaLcmBridge {
 protected:
  std::shared_ptr<lcm::LCM> send_lcm_ptr_;
  std::shared_ptr<lcm::LCM> recv_lcm_ptr_;
  std::string motion_command_channel_name_;
  std::string motion_status_channel_name_;
  std::function<void(const msg::MotionStatus&)> motion_status_callback_fn_;
  std::string control_mode_command_channel_name_;
  std::string control_mode_status_channel_name_;
  std::function<void(const msg::ControlModeParameters&)> control_mode_status_callback_fn_;

  void InternalMotionStatusLCMCallback(const lcm::ReceiveBuffer* buffer, const std::string& channel,
                                       const victor_lcm_interface::motion_status* status_msg);

  void InternalControlModeStatusLCMCallback(const lcm::ReceiveBuffer* buffer, const std::string& channel,
                                            const victor_lcm_interface::control_mode_parameters* status_msg);

 public:
  IiwaLcmBridge(const std::shared_ptr<lcm::LCM>& send_lcm_ptr, const std::shared_ptr<lcm::LCM>& recv_lcm_ptr,
                std::string  motion_command_channel_name, std::string  motion_status_channel_name,
                const std::function<void(const msg::MotionStatus&)>& motion_status_callback_fn,
                const std::string& control_mode_command_channel_name,
                const std::string& control_mode_status_channel_name,
                const std::function<void(const msg::ControlModeParameters&)>& control_mode_status_callback_fn);

  bool SendMotionCommandMessage(const msg::MotionCommand& command);

  bool SendControlModeCommandMessage(const msg::ControlModeParameters& command);
};
}  // namespace victor_hardware
