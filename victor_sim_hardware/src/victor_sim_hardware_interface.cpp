#include <algorithm>
#include <span>
#include <victor_sim_hardware/constants.hpp>
#include <victor_sim_hardware/victor_sim_hardware_interface.hpp>
#include "rclcpp/rclcpp.hpp"

static auto logger = rclcpp::get_logger("VictorSimHardwareInterface");

namespace victor_sim_hardware {
CallbackReturn VictorSimHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  enable_left_arm_ = true;
  enable_right_arm_ = true;
  RCLCPP_INFO(logger, "enable_left_arm_: %d", enable_left_arm_);
  RCLCPP_INFO(logger, "enable_right_arm_: %d", enable_right_arm_);
  RCLCPP_INFO(logger, "Number of joints: %zu", info_.joints.size());
  hw_states_position_.resize(info_.joints.size(), 0.0);
  hw_states_velocity_.resize(info_.joints.size(), 0.0);
  hw_states_effort_.resize(info_.joints.size(), 0.0);
  hw_states_external_effort_.resize(info_.joints.size(), 0.0);
  hw_states_cmd_position_.resize(info_.joints.size(), 0.0);
  hw_states_external_torque_sensor_.resize(info_.joints.size(), 0.0);

  RCLCPP_INFO(logger, "===================================================================================");
  if (enable_left_arm_ && enable_right_arm_) {
    RCLCPP_INFO(logger, "Please start the Python simulator for BOTH arms!");
  } else if (enable_left_arm_) {
    RCLCPP_INFO(logger, "Please start the Python simulator for LEFT arm!");
  } else if (enable_right_arm_) {
    RCLCPP_INFO(logger, "Please start the Python simulator for RIGHT arm!");
  }
  RCLCPP_INFO(logger, "===================================================================================");

  node_ = std::make_shared<rclcpp::Node>("victor_sim_hardware_interface_node");

  // Only initialize enabled arms
  if (enable_left_arm_) {
    left.on_init(node_);
  }
  if (enable_right_arm_) {
    right.on_init(node_);
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VictorSimHardwareInterface::export_state_interfaces() {
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
    if (has_state_interface(hardware_interface::HW_IF_VELOCITY)) {
      state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]);
    }
    if (has_state_interface(hardware_interface::HW_IF_EFFORT)) {
      state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]);
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

std::vector<hardware_interface::CommandInterface> VictorSimHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  left.add_command_interfaces(info_, command_interfaces);
  right.add_command_interfaces(info_, command_interfaces);
  return command_interfaces;
}

CallbackReturn VictorSimHardwareInterface::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  // RCLCPP_INFO(logger, "on_activate: Victor simulator hardware interface activated");
  
  // Try to establish connection or verify simulator availability
  if (enable_left_arm_ && !left.hasLatestMotionStatus()) {
    RCLCPP_WARN(logger, "Left arm simulator not ready during activation");
  }
  if (enable_right_arm_ && !right.hasLatestMotionStatus()) {
    RCLCPP_WARN(logger, "Right arm simulator not ready during activation");
  }
  
  return CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------------------------------
CallbackReturn VictorSimHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
  // RCLCPP_INFO(logger, "on_deactivate: Victor simulator hardware interface deactivated");
  return CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------------------------------
hardware_interface::return_type VictorSimHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) {
  // Remove throttling temporarily to see all calls
  // RCLCPP_INFO(logger, "Reading called - state: %d", get_state().id());

  // Spin the node to process callbacks (essential for receiving simulator data)
  rclcpp::spin_some(node_);
  
  if (get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_INFO(logger, "Not in ACTIVE state, returning early");
    return hardware_interface::return_type::OK;
  }

  // Check status only for enabled arms
  bool left_ready = !enable_left_arm_ || left.hasLatestMotionStatus();
  bool right_ready = !enable_right_arm_ || right.hasLatestMotionStatus();

  // Always set default safe values for all joints first
  std::fill(hw_states_position_.begin(), hw_states_position_.end(), 0.0);
  std::fill(hw_states_velocity_.begin(), hw_states_velocity_.end(), 0.0);
  std::fill(hw_states_effort_.begin(), hw_states_effort_.end(), 0.0);
  std::fill(hw_states_external_effort_.begin(), hw_states_external_effort_.end(), 0.0);
  std::fill(hw_states_external_torque_sensor_.begin(), hw_states_external_torque_sensor_.end(), 0.0);

  // If simulator data isn't ready yet, just return with safe default values
  if (!left_ready || !right_ready) {
    // RCLCPP_INFO(logger, "Simulator data not ready: left_ready=%d, right_ready=%d", 
    //             left_ready, right_ready);
    return hardware_interface::return_type::OK;
  }

  // Update left arm state if enabled
  if (enable_left_arm_) {
    auto const& left_motion_status = left.getLatestMotionStatus();
    
    // Left arm joint positions
    hw_states_position_[0] = left.getJoint1Position(left_motion_status.measured_joint_position);
    hw_states_position_[1] = left.getJoint2Position(left_motion_status.measured_joint_position);
    hw_states_position_[2] = left.getJoint3Position(left_motion_status.measured_joint_position);
    hw_states_position_[3] = left.getJoint4Position(left_motion_status.measured_joint_position);
    hw_states_position_[4] = left.getJoint5Position(left_motion_status.measured_joint_position);
    hw_states_position_[5] = left.getJoint6Position(left_motion_status.measured_joint_position);
    hw_states_position_[6] = left.getJoint7Position(left_motion_status.measured_joint_position);

    // Left arm joint velocities
    hw_states_velocity_[0] = left.getJoint1Position(left_motion_status.measured_joint_velocity);
    hw_states_velocity_[1] = left.getJoint2Position(left_motion_status.measured_joint_velocity);
    hw_states_velocity_[2] = left.getJoint3Position(left_motion_status.measured_joint_velocity);
    hw_states_velocity_[3] = left.getJoint4Position(left_motion_status.measured_joint_velocity);
    hw_states_velocity_[4] = left.getJoint5Position(left_motion_status.measured_joint_velocity);
    hw_states_velocity_[5] = left.getJoint6Position(left_motion_status.measured_joint_velocity);
    hw_states_velocity_[6] = left.getJoint7Position(left_motion_status.measured_joint_velocity);

    // Left arm joint efforts  
    hw_states_effort_[0] = left.getJoint1Position(left_motion_status.measured_joint_torque);
    hw_states_effort_[1] = left.getJoint2Position(left_motion_status.measured_joint_torque);
    hw_states_effort_[2] = left.getJoint3Position(left_motion_status.measured_joint_torque);
    hw_states_effort_[3] = left.getJoint4Position(left_motion_status.measured_joint_torque);
    hw_states_effort_[4] = left.getJoint5Position(left_motion_status.measured_joint_torque);
    hw_states_effort_[5] = left.getJoint6Position(left_motion_status.measured_joint_torque);
    hw_states_effort_[6] = left.getJoint7Position(left_motion_status.measured_joint_torque);

    // Left arm external torques
    hw_states_external_torque_sensor_[0] = left.getJoint1Position(left_motion_status.estimated_external_torque);
    hw_states_external_torque_sensor_[1] = left.getJoint2Position(left_motion_status.estimated_external_torque);
    hw_states_external_torque_sensor_[2] = left.getJoint3Position(left_motion_status.estimated_external_torque);
    hw_states_external_torque_sensor_[3] = left.getJoint4Position(left_motion_status.estimated_external_torque);
    hw_states_external_torque_sensor_[4] = left.getJoint5Position(left_motion_status.estimated_external_torque);
    hw_states_external_torque_sensor_[5] = left.getJoint6Position(left_motion_status.estimated_external_torque);
    hw_states_external_torque_sensor_[6] = left.getJoint7Position(left_motion_status.estimated_external_torque);

    // Left gripper state (if available) - use position values directly for simulator
    if (left.hasLatestGripperStatus()) {
      auto const& left_gripper_status = left.getLatestGripperStatus();
      // For simulator, use the position values directly from the gripper status
      // Finger A joints (assuming 3 joints per finger)
      hw_states_position_[14] = left_gripper_status.finger_a_status.position;
      hw_states_position_[15] = left_gripper_status.finger_a_status.position * 0.5; // scaled
      hw_states_position_[16] = left_gripper_status.finger_a_status.position * 0.3; // scaled
      // Scissor joint
      hw_states_position_[17] = left_gripper_status.scissor_status.position;
      // Finger B joints
      hw_states_position_[18] = left_gripper_status.finger_b_status.position;
      hw_states_position_[19] = left_gripper_status.finger_b_status.position * 0.5;
      hw_states_position_[20] = left_gripper_status.finger_b_status.position * 0.3;
      // Negative scissor joint for opposite side
      hw_states_position_[21] = -left_gripper_status.scissor_status.position;
      // Finger C joints
      hw_states_position_[22] = left_gripper_status.finger_c_status.position;
      hw_states_position_[23] = left_gripper_status.finger_c_status.position * 0.5;
      hw_states_position_[24] = left_gripper_status.finger_c_status.position * 0.3;
    }
    
    left.read_motion_status(left_motion_status);
  }

  // Update right arm state if enabled
  if (enable_right_arm_) {
    auto const& right_motion_status = right.getLatestMotionStatus();
    
    // Right arm joint positions
    hw_states_position_[7] = right.getJoint1Position(right_motion_status.measured_joint_position);
    hw_states_position_[8] = right.getJoint2Position(right_motion_status.measured_joint_position);
    hw_states_position_[9] = right.getJoint3Position(right_motion_status.measured_joint_position);
    hw_states_position_[10] = right.getJoint4Position(right_motion_status.measured_joint_position);
    hw_states_position_[11] = right.getJoint5Position(right_motion_status.measured_joint_position);
    hw_states_position_[12] = right.getJoint6Position(right_motion_status.measured_joint_position);
    hw_states_position_[13] = right.getJoint7Position(right_motion_status.measured_joint_position);

    // Right arm joint velocities
    hw_states_velocity_[7] = right.getJoint1Position(right_motion_status.measured_joint_velocity);
    hw_states_velocity_[8] = right.getJoint2Position(right_motion_status.measured_joint_velocity);
    hw_states_velocity_[9] = right.getJoint3Position(right_motion_status.measured_joint_velocity);
    hw_states_velocity_[10] = right.getJoint4Position(right_motion_status.measured_joint_velocity);
    hw_states_velocity_[11] = right.getJoint5Position(right_motion_status.measured_joint_velocity);
    hw_states_velocity_[12] = right.getJoint6Position(right_motion_status.measured_joint_velocity);
    hw_states_velocity_[13] = right.getJoint7Position(right_motion_status.measured_joint_velocity);

    // Right arm joint efforts
    hw_states_effort_[7] = right.getJoint1Position(right_motion_status.measured_joint_torque);
    hw_states_effort_[8] = right.getJoint2Position(right_motion_status.measured_joint_torque);
    hw_states_effort_[9] = right.getJoint3Position(right_motion_status.measured_joint_torque);
    hw_states_effort_[10] = right.getJoint4Position(right_motion_status.measured_joint_torque);
    hw_states_effort_[11] = right.getJoint5Position(right_motion_status.measured_joint_torque);
    hw_states_effort_[12] = right.getJoint6Position(right_motion_status.measured_joint_torque);
    hw_states_effort_[13] = right.getJoint7Position(right_motion_status.measured_joint_torque);

    // Right arm external torques
    hw_states_external_torque_sensor_[7] = right.getJoint1Position(right_motion_status.estimated_external_torque);
    hw_states_external_torque_sensor_[8] = right.getJoint2Position(right_motion_status.estimated_external_torque);
    hw_states_external_torque_sensor_[9] = right.getJoint3Position(right_motion_status.estimated_external_torque);
    hw_states_external_torque_sensor_[10] = right.getJoint4Position(right_motion_status.estimated_external_torque);
    hw_states_external_torque_sensor_[11] = right.getJoint5Position(right_motion_status.estimated_external_torque);
    hw_states_external_torque_sensor_[12] = right.getJoint6Position(right_motion_status.estimated_external_torque);
    hw_states_external_torque_sensor_[13] = right.getJoint7Position(right_motion_status.estimated_external_torque);

    // Right gripper state (if available) - use position values directly for simulator
    if (right.hasLatestGripperStatus()) {
      auto const& right_gripper_status = right.getLatestGripperStatus();
      // For simulator, use the position values directly from the gripper status
      // Finger A joints (assuming 3 joints per finger)
      hw_states_position_[25] = right_gripper_status.finger_a_status.position;
      hw_states_position_[26] = right_gripper_status.finger_a_status.position * 0.5; // scaled
      hw_states_position_[27] = right_gripper_status.finger_a_status.position * 0.3; // scaled
      // Scissor joint
      hw_states_position_[28] = right_gripper_status.scissor_status.position;
      // Finger B joints
      hw_states_position_[29] = right_gripper_status.finger_b_status.position;
      hw_states_position_[30] = right_gripper_status.finger_b_status.position * 0.5;
      hw_states_position_[31] = right_gripper_status.finger_b_status.position * 0.3;
      // Negative scissor joint for opposite side
      hw_states_position_[32] = -right_gripper_status.scissor_status.position;
      // Finger C joints
      hw_states_position_[33] = right_gripper_status.finger_c_status.position;
      hw_states_position_[34] = right_gripper_status.finger_c_status.position * 0.5;
      hw_states_position_[35] = right_gripper_status.finger_c_status.position * 0.3;
    }
    
    right.read_motion_status(right_motion_status);
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type VictorSimHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/) {
  // Add debug logging to write method too
  // RCLCPP_INFO(logger, "Write called - state: %d", get_state().id());
  
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
hardware_interface::return_type VictorSimHardwareInterface::prepare_command_mode_switch(
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
hardware_interface::return_type VictorSimHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) {
  if (enable_left_arm_) {
    left.perform_command_mode_switch(start_interfaces);
  }
  if (enable_right_arm_) {
    right.perform_command_mode_switch(start_interfaces);
  }
  return SystemInterface::perform_command_mode_switch(start_interfaces, stop_interfaces);
}

}  // namespace victor_sim_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(victor_sim_hardware::VictorSimHardwareInterface, hardware_interface::SystemInterface)
