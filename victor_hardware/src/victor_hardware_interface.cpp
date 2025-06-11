#include <algorithm>
#include <robotiq_3f_transmission_plugins/individual_control_transmission.hpp>
#include <span>
#include <victor_hardware/constants.hpp>
#include <victor_hardware/victor_hardware_interface.hpp>

#include "rclcpp/rclcpp.hpp"

static auto logger = rclcpp::get_logger("VictorHardwareInterface");

namespace victor_hardware {
CallbackReturn VictorHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  enable_left_arm_ = true;
  enable_right_arm_ = true;
  RCLCPP_INFO(logger, "enable_left_arm_: %d", enable_left_arm_);
  RCLCPP_INFO(logger, "enable_right_arm_: %d", enable_right_arm_);
  RCLCPP_INFO(logger, "Number of joints: %zu", info_.joints.size());
  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_external_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_cmd_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_external_torque_sensor_.resize(info_.joints.size(), 0);

  RCLCPP_INFO(logger, "===================================================================================");
  if (enable_left_arm_ && enable_right_arm_) {
    RCLCPP_INFO(logger, "Please start the LCMRobotInterface application on BOTH pendants!");
  } else if (enable_left_arm_) {
    RCLCPP_INFO(logger, "Please start the LCMRobotInterface application on LEFT pendant!");
  } else if (enable_right_arm_) {
    RCLCPP_INFO(logger, "Please start the LCMRobotInterface application on RIGHT pendant!");
  }
  RCLCPP_INFO(logger, "===================================================================================");

  node_ = std::make_shared<rclcpp::Node>("victor_hardware_interface_node");

  setter_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  executor_ = std::make_shared<AsyncExecutor>();
  executor_->add_node(node_);

  // Only initialize enabled arms
  if (enable_left_arm_) {
    left.on_init(node_, LEFT_SEND_PROVIDER, LEFT_RECV_PROVIDER);
  }
  if (enable_right_arm_) {
    right.on_init(node_, RIGHT_SEND_PROVIDER, RIGHT_RECV_PROVIDER);
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VictorHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // export state interfaces for joints
  for (uint i = 0; i < info_.joints.size(); i++) {
    auto joint = info_.joints[i];
    auto has_state_interface = [&](std::string const& name) {
      return std::any_of(joint.state_interfaces.begin(), joint.state_interfaces.end(),
                         [&](auto const& state_interface) { return state_interface.name == name; });
    };

    if (has_state_interface(hardware_interface::HW_IF_POSITION)) {
      state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]);
    }
    if (has_state_interface(hardware_interface::HW_IF_EFFORT)) {
      state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &hw_states_external_effort_[i]);
    }
    if (has_state_interface(EXTERNAL_TORQUE)) {
      state_interfaces.emplace_back(joint.name, EXTERNAL_TORQUE, &hw_states_external_torque_sensor_[i]);
    }
    if (has_state_interface(COMMANDED_POSITION)) {
      state_interfaces.emplace_back(joint.name, COMMANDED_POSITION, &hw_states_cmd_position_[i]);
    }
  }

  left.add_state_interfaces(state_interfaces);
  right.add_state_interfaces(state_interfaces);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VictorHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  left.add_command_interfaces(info_, command_interfaces);
  right.add_command_interfaces(info_, command_interfaces);
  return command_interfaces;
}

CallbackReturn VictorHardwareInterface::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  lcm_thread_running_ = true;
  lcm_thread_ = std::thread(&VictorHardwareInterface::LCMThread, this);

  RCLCPP_INFO(logger, "on_activate: LCM processing thread started");

  return CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------------------------------
CallbackReturn VictorHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
  // Stop the LCM processing thread
  RCLCPP_INFO(logger, "on_deactivate: Stopping LCM processing thread");

  lcm_thread_running_ = false;
  lcm_thread_.join();

  RCLCPP_INFO(logger, "LCM thread stopped.");

  return CallbackReturn::SUCCESS;
}

void VictorHardwareInterface::LCMThread() {
  while (lcm_thread_running_) {
    if (enable_left_arm_) {
      left.recv_lcm_ptr_->handleTimeout(1000);
    }
    if (enable_right_arm_) {
      right.recv_lcm_ptr_->handleTimeout(1000);
    }
  }
}

// ------------------------------------------------------------------------------------------
hardware_interface::return_type VictorHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) {
  if (get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return hardware_interface::return_type::OK;
  }

  // Check status only for enabled arms
  bool left_ready = !enable_left_arm_ || 
                   (left.motion_status_listener_->hasLatestMessage() && 
                    left.gripper_status_listener_->hasLatestMessage());
  bool right_ready = !enable_right_arm_ || 
                    (right.motion_status_listener_->hasLatestMessage() && 
                     right.gripper_status_listener_->hasLatestMessage());

  if (!left_ready || !right_ready) {
    return hardware_interface::return_type::OK;
  }

  // Set default values for all joints
  std::fill(hw_states_position_.begin(), hw_states_position_.end(), 0.0);
  std::fill(hw_states_external_effort_.begin(), hw_states_external_effort_.end(), 0.0);
  std::fill(hw_states_external_torque_sensor_.begin(), hw_states_external_torque_sensor_.end(), 0.0);

  // Update left arm state if enabled
  if (enable_left_arm_) {
    auto const& left_motion_status = left.motion_status_listener_->getLatestMessage();
    
    // Left arm joint positions
    hw_states_position_[0] = left_motion_status.measured_joint_position.joint_1;
    hw_states_position_[1] = left_motion_status.measured_joint_position.joint_2;
    hw_states_position_[2] = left_motion_status.measured_joint_position.joint_3;
    hw_states_position_[3] = left_motion_status.measured_joint_position.joint_4;
    hw_states_position_[4] = left_motion_status.measured_joint_position.joint_5;
    hw_states_position_[5] = left_motion_status.measured_joint_position.joint_6;
    hw_states_position_[6] = left_motion_status.measured_joint_position.joint_7;

    // Left arm joint efforts
    hw_states_external_effort_[0] = left_motion_status.measured_joint_torque.joint_1;
    hw_states_external_effort_[1] = left_motion_status.measured_joint_torque.joint_2;
    hw_states_external_effort_[2] = left_motion_status.measured_joint_torque.joint_3;
    hw_states_external_effort_[3] = left_motion_status.measured_joint_torque.joint_4;
    hw_states_external_effort_[4] = left_motion_status.measured_joint_torque.joint_5;
    hw_states_external_effort_[5] = left_motion_status.measured_joint_torque.joint_6;
    hw_states_external_effort_[6] = left_motion_status.measured_joint_torque.joint_7;

    // Left arm external torques
    hw_states_external_torque_sensor_[0] = left_motion_status.estimated_external_torque.joint_1;
    hw_states_external_torque_sensor_[1] = left_motion_status.estimated_external_torque.joint_2;
    hw_states_external_torque_sensor_[2] = left_motion_status.estimated_external_torque.joint_3;
    hw_states_external_torque_sensor_[3] = left_motion_status.estimated_external_torque.joint_4;
    hw_states_external_torque_sensor_[4] = left_motion_status.estimated_external_torque.joint_5;
    hw_states_external_torque_sensor_[5] = left_motion_status.estimated_external_torque.joint_6;
    hw_states_external_torque_sensor_[6] = left_motion_status.estimated_external_torque.joint_7;

    // Left gripper state
    auto const& left_gripper_status = left.gripper_status_listener_->getLatestMessage();
    auto const& left_finger_a_pos =
        robotiq_3f_transmission_plugins::double_to_uint8(left_gripper_status.finger_a_status.position);
    auto const& left_finger_b_pos =
        robotiq_3f_transmission_plugins::double_to_uint8(left_gripper_status.finger_b_status.position);
    auto const& left_finger_c_pos =
        robotiq_3f_transmission_plugins::double_to_uint8(left_gripper_status.finger_c_status.position);
    auto const& left_scissor_pos =
        robotiq_3f_transmission_plugins::double_to_uint8(left_gripper_status.scissor_status.position);
    auto const& left_finger_a_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(left_finger_a_pos);
    auto const& left_finger_b_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(left_finger_b_pos);
    auto const& left_finger_c_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(left_finger_c_pos);
    auto const& left_scissor_theta = robotiq_3f_transmission_plugins::get_palm_finger_pos(left_scissor_pos);
    hw_states_position_[14] = left_finger_a_thetas[0];
    hw_states_position_[15] = left_finger_a_thetas[1];
    hw_states_position_[16] = left_finger_a_thetas[2];
    hw_states_position_[17] = left_scissor_theta;
    hw_states_position_[18] = left_finger_b_thetas[0];
    hw_states_position_[19] = left_finger_b_thetas[1];
    hw_states_position_[20] = left_finger_b_thetas[2];
    hw_states_position_[21] = -left_scissor_theta;
    hw_states_position_[22] = left_finger_c_thetas[0];
    hw_states_position_[23] = left_finger_c_thetas[1];
    hw_states_position_[24] = left_finger_c_thetas[2];
    
    left.read_motion_status(left_motion_status);
  }

  // Update right arm state if enabled
  if (enable_right_arm_) {
    auto const& right_motion_status = right.motion_status_listener_->getLatestMessage();
    
    // Right arm joint positions
    hw_states_position_[7] = right_motion_status.measured_joint_position.joint_1;
    hw_states_position_[8] = right_motion_status.measured_joint_position.joint_2;
    hw_states_position_[9] = right_motion_status.measured_joint_position.joint_3;
    hw_states_position_[10] = right_motion_status.measured_joint_position.joint_4;
    hw_states_position_[11] = right_motion_status.measured_joint_position.joint_5;
    hw_states_position_[12] = right_motion_status.measured_joint_position.joint_6;
    hw_states_position_[13] = right_motion_status.measured_joint_position.joint_7;

    // Right arm joint efforts
    hw_states_external_effort_[7] = right_motion_status.measured_joint_torque.joint_1;
    hw_states_external_effort_[8] = right_motion_status.measured_joint_torque.joint_2;
    hw_states_external_effort_[9] = right_motion_status.measured_joint_torque.joint_3;
    hw_states_external_effort_[10] = right_motion_status.measured_joint_torque.joint_4;
    hw_states_external_effort_[11] = right_motion_status.measured_joint_torque.joint_5;
    hw_states_external_effort_[12] = right_motion_status.measured_joint_torque.joint_6;
    hw_states_external_effort_[13] = right_motion_status.measured_joint_torque.joint_7;

    // Right arm external torques
    hw_states_external_torque_sensor_[7] = right_motion_status.estimated_external_torque.joint_1;
    hw_states_external_torque_sensor_[8] = right_motion_status.estimated_external_torque.joint_2;
    hw_states_external_torque_sensor_[9] = right_motion_status.estimated_external_torque.joint_3;
    hw_states_external_torque_sensor_[10] = right_motion_status.estimated_external_torque.joint_4;
    hw_states_external_torque_sensor_[11] = right_motion_status.estimated_external_torque.joint_5;
    hw_states_external_torque_sensor_[12] = right_motion_status.estimated_external_torque.joint_6;
    hw_states_external_torque_sensor_[13] = right_motion_status.estimated_external_torque.joint_7;

    // Right gripper state
    auto const& right_gripper_status = right.gripper_status_listener_->getLatestMessage();
    auto const& right_finger_a_pos =
        robotiq_3f_transmission_plugins::double_to_uint8(right_gripper_status.finger_a_status.position);
    auto const& right_finger_b_pos =
        robotiq_3f_transmission_plugins::double_to_uint8(right_gripper_status.finger_b_status.position);
    auto const& right_finger_c_pos =
        robotiq_3f_transmission_plugins::double_to_uint8(right_gripper_status.finger_c_status.position);
    auto const& right_scissor_pos =
        robotiq_3f_transmission_plugins::double_to_uint8(right_gripper_status.scissor_status.position);
    auto const& right_finger_a_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(right_finger_a_pos);
    auto const& right_finger_b_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(right_finger_b_pos);
    auto const& right_finger_c_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(right_finger_c_pos);
    auto const& right_scissor_theta = robotiq_3f_transmission_plugins::get_palm_finger_pos(right_scissor_pos);
    hw_states_position_[25] = right_finger_a_thetas[0];
    hw_states_position_[26] = right_finger_a_thetas[1];
    hw_states_position_[27] = right_finger_a_thetas[2];
    hw_states_position_[28] = right_scissor_theta;
    hw_states_position_[29] = right_finger_b_thetas[0];
    hw_states_position_[30] = right_finger_b_thetas[1];
    hw_states_position_[31] = right_finger_b_thetas[2];
    hw_states_position_[32] = -right_scissor_theta;
    hw_states_position_[33] = right_finger_c_thetas[0];
    hw_states_position_[34] = right_finger_c_thetas[1];
    hw_states_position_[35] = right_finger_c_thetas[2];
    
    right.read_motion_status(right_motion_status);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VictorHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/) {
  if (get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type left_return = hardware_interface::return_type::OK;
  hardware_interface::return_type right_return = hardware_interface::return_type::OK;

  if (enable_left_arm_) {
    left_return = left.send_motion_command();
  }
  if (enable_right_arm_) {
    right_return = right.send_motion_command();
  }

  if (left_return != hardware_interface::return_type::OK || right_return != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}
hardware_interface::return_type VictorHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) {
  // Check for errors:
  //  - no control mode interfaces are claimed, that's likely an error
  auto is_control_mode_interface = [&](std::string const& interface) {
    // check if the interface name contains any of the control mode interface names
    return (interface.find(JOINT_POSITION_INTERFACE) != std::string::npos ||
            interface.find(JOINT_IMPEDANCE_INTERFACE) != std::string::npos ||
            interface.find(CARTESIAN_POSE_INTERFACE) != std::string::npos ||
            interface.find(CARTESIAN_IMPEDANCE_INTERFACE) != std::string::npos);
  };
  bool has_control_mode_interface =
      std::any_of(start_interfaces.cbegin(), start_interfaces.cend(), is_control_mode_interface);

  if (!has_control_mode_interface && !start_interfaces.empty()) {
    RCLCPP_WARN(logger, "No control mode interface claimed, likely an error.");
    RCLCPP_WARN(logger, "start_interfaces:");
    for (auto const& iface : start_interfaces) {
      RCLCPP_WARN_STREAM(logger, "\t" << iface);
    }
  }

  return SystemInterface::prepare_command_mode_switch(start_interfaces, stop_interfaces);
}
hardware_interface::return_type VictorHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) {
  if (enable_left_arm_) {
    left.perform_command_mode_switch(start_interfaces);
  }
  if (enable_right_arm_) {
    right.perform_command_mode_switch(start_interfaces);
  }
  return SystemInterface::perform_command_mode_switch(start_interfaces, stop_interfaces);
}

}  // namespace victor_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(victor_hardware::VictorHardwareInterface, hardware_interface::SystemInterface)
