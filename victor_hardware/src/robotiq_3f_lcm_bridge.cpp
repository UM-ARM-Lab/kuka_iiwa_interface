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


}
