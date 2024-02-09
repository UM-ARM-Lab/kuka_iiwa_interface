
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The class that does the actual communication
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

IiwaLcmBridge::IiwaLcmBridge(
    const std::shared_ptr<lcm::LCM>& send_lcm_ptr, const std::shared_ptr<lcm::LCM>& recv_lcm_ptr,
    std::string motion_command_channel_name, std::string motion_status_channel_name,
    const std::function<void(const msg::MotionStatus&)>& motion_status_callback_fn,
    const std::string& control_mode_command_channel_name, const std::string& control_mode_status_channel_name,
    const std::function<void(const msg::ControlModeParameters&)>& control_mode_status_callback_fn)
    : send_lcm_ptr_(send_lcm_ptr),
      recv_lcm_ptr_(recv_lcm_ptr),
      motion_command_channel_name_(std::move(motion_command_channel_name)),
      motion_status_channel_name_(std::move(motion_status_channel_name)),
      motion_status_callback_fn_(motion_status_callback_fn),
      control_mode_command_channel_name_(control_mode_command_channel_name),
      control_mode_status_channel_name_(control_mode_status_channel_name),
      control_mode_status_callback_fn_(control_mode_status_callback_fn) {
  // Check that the LCM objects are ready to communicate before using them
  if (send_lcm_ptr_->good() != true) {
    throw std::invalid_argument("Send LCM interface is not good");
  }
  if (recv_lcm_ptr_->good() != true) {
    throw std::invalid_argument("Receive LCM interface is not good");
  }
  recv_lcm_ptr_->subscribe(motion_status_channel_name_, &IiwaLcmBridge::InternalMotionStatusLCMCallback, this);
  recv_lcm_ptr_->subscribe(control_mode_status_channel_name_, &IiwaLcmBridge::InternalControlModeStatusLCMCallback,
                           this);
}

bool IiwaLcmBridge::SendMotionCommandMessage(const msg::MotionCommand& command) {
  const victor_lcm_interface::motion_command lcm_command = motionCommandRosToLcm(command);
  const int ret = send_lcm_ptr_->publish(motion_command_channel_name_, &lcm_command);
  if (ret == 0) {
    return true;
  } else {
    return false;
  }
}

bool IiwaLcmBridge::SendControlModeCommandMessage(const msg::ControlModeParameters& command) {
  const victor_lcm_interface::control_mode_parameters lcm_command = controlModeParamsRosToLcm(command);
  const int ret = send_lcm_ptr_->publish(control_mode_command_channel_name_, &lcm_command);
  if (ret == 0) {
    return true;
  } else {
    return false;
  }
}

void IiwaLcmBridge::InternalMotionStatusLCMCallback(const lcm::ReceiveBuffer* /*buffer*/,
                                                    const std::string& /*channel*/,
                                                    const victor_lcm_interface::motion_status* status_msg) {
  const msg::MotionStatus ros_status = motionStatusLcmToRos(*status_msg);
  motion_status_callback_fn_(ros_status);
}

void IiwaLcmBridge::InternalControlModeStatusLCMCallback(
    const lcm::ReceiveBuffer* /*buffer*/, const std::string& /*channel*/,
    const victor_lcm_interface::control_mode_parameters* status_msg) {
  const msg::ControlModeParameters ros_status = controlModeParamsLcmToRos(*status_msg);
  control_mode_status_callback_fn_(ros_status);
}

}  // namespace victor_hardware
