#pragma once

// ROS message headers
#include <victor_hardware_interfaces/msg/robotiq3_finger_command.hpp>
#include <victor_hardware_interfaces/msg/robotiq3_finger_status.hpp>

// LCM type headers
#include <victor_lcm_interface/robotiq_3finger_command.hpp>
#include <victor_lcm_interface/robotiq_3finger_status.hpp>
// LCM
#include <lcm/lcm-cpp.hpp>

namespace victor_hardware {
namespace msgs = victor_hardware_interfaces::msg;

/////////////////////////////////////////////////////////////////////////////////
// Robotiq3FingerHardwardInterface class declaration
/////////////////////////////////////////////////////////////////////////////////

class Robotiq3fLcmBridge {
 protected:
  std::shared_ptr<lcm::LCM> send_lcm_ptr_;
  std::shared_ptr<lcm::LCM> recv_lcm_ptr_;
  std::string command_channel_name_;
  std::string status_channel_name_;
  std::function<void(const msgs::Robotiq3FingerStatus&)> status_callback_fn_;

  void internalStatusLCMCallback(const lcm::ReceiveBuffer* buffer, const std::string& channel,
                                 const victor_lcm_interface::robotiq_3finger_status* status_msg);

 public:
  Robotiq3fLcmBridge(std::shared_ptr<lcm::LCM> send_lcm_ptr, std::shared_ptr<lcm::LCM> recv_lcm_ptr,
                     std::string command_channel_name, std::string status_channel_name,
                     std::function<void(const msgs::Robotiq3FingerStatus&)> status_callback_fn);

  bool sendCommandMessage(const msgs::Robotiq3FingerCommand& command);
};

/////////////////////////////////////////////////////////////////////////////////
// Ros and LCM convert helper functions
/////////////////////////////////////////////////////////////////////////////////

victor_lcm_interface::robotiq_3finger_actuator_command fingerCommandRosToLcm(
    const msgs::Robotiq3FingerActuatorCommand& finger_command);

msgs::Robotiq3FingerActuatorStatus fingerStatusLcmToRos(
    const victor_lcm_interface::robotiq_3finger_actuator_status& finger_status);

msgs::Robotiq3FingerObjectStatus objectStatusLcmToRos(
    const victor_lcm_interface::robotiq_3finger_object_status& object_status);

msgs::Robotiq3FingerStatus statusLcmToRos(const victor_lcm_interface::robotiq_3finger_status& status);

victor_lcm_interface::robotiq_3finger_command commandRosToLcm(const msgs::Robotiq3FingerCommand& command);

}  // namespace victor_hardware
