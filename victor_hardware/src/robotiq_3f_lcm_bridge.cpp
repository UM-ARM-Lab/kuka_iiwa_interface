#include <arm_utilities/arm_helpers.hpp>
#include <arm_utilities/ros_helpers.hpp>
#include <rclcpp/time.hpp>
#include <utility>
#include <victor_hardware/robotiq_3f_lcm_bridge.hpp>

namespace msgs = victor_hardware_interfaces::msg;
namespace victor_hardware {

/////////////////////////////////////////////////////////////////////////////////
// Robotiq3FingerHardwardInterface class implementation
/////////////////////////////////////////////////////////////////////////////////

Robotiq3fLcmBridge::Robotiq3fLcmBridge(std::shared_ptr<lcm::LCM> send_lcm_ptr, std::shared_ptr<lcm::LCM> recv_lcm_ptr,
                                       std::string command_channel_name, std::string status_channel_name,
                                       std::function<void(const msgs::Robotiq3FingerStatus&)> status_callback_fn)
    : send_lcm_ptr_(std::move(send_lcm_ptr)),
      recv_lcm_ptr_(std::move(recv_lcm_ptr)),
      command_channel_name_(std::move(command_channel_name)),
      status_channel_name_(std::move(status_channel_name)),
      status_callback_fn_(std::move(status_callback_fn)) {
  // Check lcm is valid to communicating
  if (!send_lcm_ptr_->good()) {
    throw std::invalid_argument("Send LCM interface is not good");
  }
  if (!recv_lcm_ptr_->good()) {
    throw std::invalid_argument("Receive LCM interface is not good");
  }
  recv_lcm_ptr_->subscribe(status_channel_name_, &Robotiq3fLcmBridge::internalStatusLCMCallback, this);
}

bool Robotiq3fLcmBridge::sendCommandMessage(const msgs::Robotiq3FingerCommand& command) {
  const victor_lcm_interface::robotiq_3finger_command lcm_command = commandRosToLcm(command);
  const int ret = send_lcm_ptr_->publish(command_channel_name_, &lcm_command);
  if (ret == 0) {
    return true;
  } else {
    return false;
  }
}

void Robotiq3fLcmBridge::internalStatusLCMCallback(const lcm::ReceiveBuffer* /*buffer*/, const std::string& /*channel*/,
                                                   const victor_lcm_interface::robotiq_3finger_status* status_msg) {
  const msgs::Robotiq3FingerStatus ros_status = statusLcmToRos(*status_msg);
  status_callback_fn_(ros_status);
}

/////////////////////////////////////////////////////////////////////////////////
// Ros and LCM convert helper functions
/////////////////////////////////////////////////////////////////////////////////

victor_lcm_interface::robotiq_3finger_actuator_command fingerCommandRosToLcm(
    const msgs::Robotiq3FingerActuatorCommand& finger_command) {
  victor_lcm_interface::robotiq_3finger_actuator_command lcm_command{};

  // "Magic" number 0.0 and 1 are the bounds representing percentage value
  lcm_command.position = arm_helpers::ClampValueAndWarn(finger_command.position, 0.0, 1.0);
  lcm_command.speed = arm_helpers::ClampValueAndWarn(finger_command.speed, 0.0, 1.0);
  lcm_command.force = arm_helpers::ClampValueAndWarn(finger_command.force, 0.0, 1.0);
  lcm_command.timestamp = rclcpp::Time(finger_command.header.stamp).seconds();
  return lcm_command;
}

msgs::Robotiq3FingerActuatorStatus fingerStatusLcmToRos(
    const victor_lcm_interface::robotiq_3finger_actuator_status& finger_status) {
  msgs::Robotiq3FingerActuatorStatus ros_status;
  ros_status.position = finger_status.position;
  ros_status.position_request = finger_status.position_request;
  ros_status.current = finger_status.current;
  ros_status.header.stamp = ros_helpers::secondsToTimeMsg(finger_status.timestamp);
  return ros_status;
}

rclcpp::Time secondsToStamp(const victor_lcm_interface::robotiq_3finger_object_status& object_status);
msgs::Robotiq3FingerObjectStatus objectStatusLcmToRos(
    const victor_lcm_interface::robotiq_3finger_object_status& object_status) {
  msgs::Robotiq3FingerObjectStatus ros_status;
  ros_status.status = (uint8_t)object_status.status;
  ros_status.header.stamp = ros_helpers::secondsToTimeMsg(object_status.timestamp);
  return ros_status;
}

msgs::Robotiq3FingerStatus statusLcmToRos(const victor_lcm_interface::robotiq_3finger_status& status) {
  msgs::Robotiq3FingerStatus ros_status;
  ros_status.finger_a_status = fingerStatusLcmToRos(status.finger_a_status);
  ros_status.finger_b_status = fingerStatusLcmToRos(status.finger_b_status);
  ros_status.finger_c_status = fingerStatusLcmToRos(status.finger_c_status);
  ros_status.scissor_status = fingerStatusLcmToRos(status.scissor_status);
  ros_status.finger_a_object_status = objectStatusLcmToRos(status.finger_a_object_status);
  ros_status.finger_b_object_status = objectStatusLcmToRos(status.finger_b_object_status);
  ros_status.finger_c_object_status = objectStatusLcmToRos(status.finger_c_object_status);
  ros_status.scissor_object_status = objectStatusLcmToRos(status.scissor_object_status);
  ros_status.gripper_action_status = (uint8_t)status.gripper_action_status;
  ros_status.gripper_system_status = (uint8_t)status.gripper_system_status;
  ros_status.gripper_motion_status = (uint8_t)status.gripper_motion_status;
  ros_status.gripper_fault_status = (uint8_t)status.gripper_fault_status;
  ros_status.initialization_status = (uint8_t)status.initialization_status;
  ros_status.header.stamp = ros_helpers::secondsToTimeMsg(status.timestamp);
  return ros_status;
}

victor_lcm_interface::robotiq_3finger_command commandRosToLcm(const msgs::Robotiq3FingerCommand& command) {
  victor_lcm_interface::robotiq_3finger_command lcm_command{};
  lcm_command.finger_a_command = fingerCommandRosToLcm(command.finger_a_command);
  lcm_command.finger_b_command = fingerCommandRosToLcm(command.finger_b_command);
  lcm_command.finger_c_command = fingerCommandRosToLcm(command.finger_c_command);
  lcm_command.scissor_command = fingerCommandRosToLcm(command.scissor_command);
  lcm_command.timestamp = rclcpp::Time(command.header.stamp.sec).seconds();
  return lcm_command;
}

}
