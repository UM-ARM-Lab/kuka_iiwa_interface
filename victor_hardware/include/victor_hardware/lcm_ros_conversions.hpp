#pragma once

#include <victor_hardware_interfaces/msg/cartesian_control_mode_limits.hpp>
#include <victor_hardware_interfaces/msg/cartesian_impedance_parameters.hpp>
#include <victor_hardware_interfaces/msg/cartesian_path_execution_parameters.hpp>
#include <victor_hardware_interfaces/msg/cartesian_value_quantity.hpp>
#include <victor_hardware_interfaces/msg/control_mode.hpp>
#include <victor_hardware_interfaces/msg/control_mode_parameters.hpp>
#include <victor_hardware_interfaces/msg/joint_impedance_parameters.hpp>
#include <victor_hardware_interfaces/msg/joint_path_execution_parameters.hpp>
#include <victor_hardware_interfaces/msg/joint_value_quantity.hpp>
#include <victor_hardware_interfaces/msg/motion_command.hpp>
#include <victor_hardware_interfaces/msg/motion_status.hpp>
#include <victor_lcm_interface/cartesian_control_mode_limits.hpp>
#include <victor_lcm_interface/cartesian_impedance_parameters.hpp>
#include <victor_lcm_interface/cartesian_path_execution_parameters.hpp>
#include <victor_lcm_interface/cartesian_value_quantity.hpp>
#include <victor_lcm_interface/control_mode.hpp>
#include <victor_lcm_interface/control_mode_parameters.hpp>
#include <victor_lcm_interface/joint_impedance_parameters.hpp>
#include <victor_lcm_interface/joint_path_execution_parameters.hpp>
#include <victor_lcm_interface/joint_value_quantity.hpp>
#include <victor_lcm_interface/motion_command.hpp>
#include <victor_lcm_interface/motion_status.hpp>


namespace msg = victor_hardware_interfaces::msg;

namespace victor_hardware {

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

msg::JointPathExecutionParameters jointPexpLcmToRos( const victor_lcm_interface::joint_path_execution_parameters& lcm_pexp);

victor_lcm_interface::joint_path_execution_parameters jointPexpRosToLcm(const msg::JointPathExecutionParameters& ros_pexp);

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

} // namespace victor_hardware