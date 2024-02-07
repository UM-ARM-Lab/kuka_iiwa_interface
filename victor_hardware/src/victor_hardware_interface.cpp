#include <algorithm>
#include <robotiq_3f_transmission_plugins/individual_control_transmission.hpp>
#include <span>
#include <victor_hardware/constants.hpp>
#include <victor_hardware/victor_hardware_interface.hpp>

#include "rclcpp/rclcpp.hpp"

auto logger = rclcpp::get_logger("VictorHardwareInterface");

namespace victor_hardware {
CallbackReturn VictorHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Some of these arrays will be bigger than they need to be, since not all joints are actuated (e.g. finger joints)
  RCLCPP_INFO_STREAM(logger, "Found " << info_.joints.size() << " joints");

  hw_pos_cmds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_external_torque_sensor_.resize(info_.joints.size(), 0);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VictorHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    auto joint = info_.joints[i];
    auto has_state_interface = [&](std::string const& name) {
      return std::any_of(joint.state_interfaces.begin(), joint.state_interfaces.end(),
                         [&](auto const& state_interface) { return state_interface.name == name; });
    };

    if (has_state_interface("position")) {
      state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]);
    }
    if (has_state_interface("effort")) {
      state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]);
    }
    if (has_state_interface("external_torque")) {
      state_interfaces.emplace_back(joint.name, "external_torque", &hw_states_external_torque_sensor_[i]);
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VictorHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    auto joint = info_.joints[i];
    auto has_cmd_interface = [&](std::string const& name) {
      return std::any_of(joint.command_interfaces.begin(), joint.command_interfaces.end(),
                         [&](auto const& cmd_interface) { return cmd_interface.name == name; });
    };

    if (has_cmd_interface("position")) {
      command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &hw_pos_cmds_[i]);
    }
  }

  return command_interfaces;
}

CallbackReturn VictorHardwareInterface::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  RCLCPP_INFO(logger, "Starting on_activate");

  // NOTE: changing these values here requires corresponding changes in the LCMRobotInterface application
  //  and also the victor_lcm_bridge launch file.
  std::string const left_recv_provider = "udp://10.10.10.108:30002";
  std::string const right_recv_provider = "udp://10.10.10.108:30001";
  std::string const left_send_provider = "udp://10.10.10.12:30000";
  std::string const right_send_provider = "udp://10.10.10.11:30000";

  left_send_lcm_ptr_ = std::make_shared<lcm::LCM>(left_send_provider);
  left_recv_lcm_ptr_ = std::make_shared<lcm::LCM>(left_recv_provider);
  right_send_lcm_ptr_ = std::make_shared<lcm::LCM>(right_send_provider);
  right_recv_lcm_ptr_ = std::make_shared<lcm::LCM>(right_recv_provider);

  RCLCPP_INFO(logger, "===================================================================================");
  RCLCPP_INFO(logger, "Please start the LCMRobotInterface application on BOTH pendants!");
  RCLCPP_INFO(logger, "===================================================================================");

  if (!left_send_lcm_ptr_->good()) {
    RCLCPP_ERROR(logger, "Left Send LCM interface is not good");
    return CallbackReturn::ERROR;
  }
  RCLCPP_DEBUG(logger, "Left send LCM interface is good");

  if (!left_recv_lcm_ptr_->good()) {
    RCLCPP_ERROR(logger, "Left Receive LCM interface is not good");
    return CallbackReturn::ERROR;
  }
  RCLCPP_DEBUG(logger, "Left receive LCM interface is good");

  if (!right_send_lcm_ptr_->good()) {
    RCLCPP_ERROR(logger, "Right Send LCM interface is not good");
    return CallbackReturn::ERROR;
  }
  RCLCPP_DEBUG(logger, "Right send LCM interface is good");

  if (!right_recv_lcm_ptr_->good()) {
    RCLCPP_ERROR(logger, "Right Receive LCM interface is not good");
    return CallbackReturn::ERROR;
  }
  RCLCPP_DEBUG(logger, "Right receive LCM interface is good");

  left_motion_status_listener_ = std::make_unique<LcmListener<victor_lcm_interface::motion_status>>(
      left_recv_lcm_ptr_, DEFAULT_MOTION_STATUS_CHANNEL);
  right_motion_status_listener_ = std::make_unique<LcmListener<victor_lcm_interface::motion_status>>(
      right_recv_lcm_ptr_, DEFAULT_MOTION_STATUS_CHANNEL);
  left_control_mode_listener_ = std::make_unique<LcmListener<victor_lcm_interface::control_mode_parameters>>(
      left_recv_lcm_ptr_, DEFAULT_CONTROL_MODE_STATUS_CHANNEL);
  right_control_mode_listener_ = std::make_unique<LcmListener<victor_lcm_interface::control_mode_parameters>>(
      right_recv_lcm_ptr_, DEFAULT_CONTROL_MODE_STATUS_CHANNEL);
  left_gripper_status_listener_ = std::make_unique<LcmListener<victor_lcm_interface::robotiq_3finger_status>>(
      left_recv_lcm_ptr_, DEFAULT_GRIPPER_STATUS_CHANNEL);
  right_gripper_status_listener_ = std::make_unique<LcmListener<victor_lcm_interface::robotiq_3finger_status>>(
      right_recv_lcm_ptr_, DEFAULT_GRIPPER_STATUS_CHANNEL);

  lcm_thread_running_ = true;
  lcm_thread_ = std::thread(&VictorHardwareInterface::LCMThread, this);

  // Send default control mode parameters, copied from victor_utils.py
  // FIXME: is this actually necessary?
  victor_lcm_interface::control_mode_parameters new_control_mode{};
  new_control_mode.control_mode.mode = victor_lcm_interface::control_mode::JOINT_POSITION;
  new_control_mode.joint_path_execution_params.joint_relative_velocity = 0.1;
  new_control_mode.joint_path_execution_params.joint_relative_acceleration = 0.1;
  new_control_mode.joint_path_execution_params.override_joint_acceleration = 0.0;

  sleep(1);

  RCLCPP_INFO(logger, "Sending default control mode parameters to both arms.");

  left_send_lcm_ptr_->publish(DEFAULT_CONTROL_MODE_COMMAND_CHANNEL, &new_control_mode);
  right_send_lcm_ptr_->publish(DEFAULT_CONTROL_MODE_COMMAND_CHANNEL, &new_control_mode);

  RCLCPP_INFO(logger, "On Activate finished successfully");

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
    left_recv_lcm_ptr_->handleTimeout(1000);
    right_recv_lcm_ptr_->handleTimeout(1000);
  }
}

// ------------------------------------------------------------------------------------------
hardware_interface::return_type VictorHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) {
  auto const& left_motion_status = left_motion_status_listener_->getLatestMessage();
  auto const& right_motion_status = right_motion_status_listener_->getLatestMessage();

  hw_states_position_[0] = left_motion_status.measured_joint_position.joint_1;
  hw_states_position_[1] = left_motion_status.measured_joint_position.joint_2;
  hw_states_position_[2] = left_motion_status.measured_joint_position.joint_3;
  hw_states_position_[3] = left_motion_status.measured_joint_position.joint_4;
  hw_states_position_[4] = left_motion_status.measured_joint_position.joint_5;
  hw_states_position_[5] = left_motion_status.measured_joint_position.joint_6;
  hw_states_position_[6] = left_motion_status.measured_joint_position.joint_7;
  hw_states_position_[7] = right_motion_status.measured_joint_position.joint_1;
  hw_states_position_[8] = right_motion_status.measured_joint_position.joint_2;
  hw_states_position_[9] = right_motion_status.measured_joint_position.joint_3;
  hw_states_position_[10] = right_motion_status.measured_joint_position.joint_4;
  hw_states_position_[11] = right_motion_status.measured_joint_position.joint_5;
  hw_states_position_[12] = right_motion_status.measured_joint_position.joint_6;
  hw_states_position_[13] = right_motion_status.measured_joint_position.joint_7;

  hw_states_effort_[0] = left_motion_status.measured_joint_torque.joint_1;
  hw_states_effort_[1] = left_motion_status.measured_joint_torque.joint_2;
  hw_states_effort_[2] = left_motion_status.measured_joint_torque.joint_3;
  hw_states_effort_[3] = left_motion_status.measured_joint_torque.joint_4;
  hw_states_effort_[4] = left_motion_status.measured_joint_torque.joint_5;
  hw_states_effort_[5] = left_motion_status.measured_joint_torque.joint_6;
  hw_states_effort_[6] = left_motion_status.measured_joint_torque.joint_7;
  hw_states_effort_[7] = right_motion_status.measured_joint_torque.joint_1;
  hw_states_effort_[8] = right_motion_status.measured_joint_torque.joint_2;
  hw_states_effort_[9] = right_motion_status.measured_joint_torque.joint_3;
  hw_states_effort_[10] = right_motion_status.measured_joint_torque.joint_4;
  hw_states_effort_[11] = right_motion_status.measured_joint_torque.joint_5;
  hw_states_effort_[12] = right_motion_status.measured_joint_torque.joint_6;
  hw_states_effort_[13] = right_motion_status.measured_joint_torque.joint_7;

  hw_states_external_torque_sensor_[0] = left_motion_status.estimated_external_torque.joint_1;
  hw_states_external_torque_sensor_[1] = left_motion_status.estimated_external_torque.joint_2;
  hw_states_external_torque_sensor_[2] = left_motion_status.estimated_external_torque.joint_3;
  hw_states_external_torque_sensor_[3] = left_motion_status.estimated_external_torque.joint_4;
  hw_states_external_torque_sensor_[4] = left_motion_status.estimated_external_torque.joint_5;
  hw_states_external_torque_sensor_[5] = left_motion_status.estimated_external_torque.joint_6;
  hw_states_external_torque_sensor_[6] = left_motion_status.estimated_external_torque.joint_7;
  hw_states_external_torque_sensor_[7] = right_motion_status.estimated_external_torque.joint_1;
  hw_states_external_torque_sensor_[8] = right_motion_status.estimated_external_torque.joint_2;
  hw_states_external_torque_sensor_[9] = right_motion_status.estimated_external_torque.joint_3;
  hw_states_external_torque_sensor_[10] = right_motion_status.estimated_external_torque.joint_4;
  hw_states_external_torque_sensor_[11] = right_motion_status.estimated_external_torque.joint_5;
  hw_states_external_torque_sensor_[12] = right_motion_status.estimated_external_torque.joint_6;
  hw_states_external_torque_sensor_[13] = right_motion_status.estimated_external_torque.joint_7;

  // now also fill out the positions for the finger joints
  auto const& left_gripper_status = left_gripper_status_listener_->getLatestMessage();
  auto const& left_finger_a_pos = left_gripper_status.finger_a_status.position;
  auto const& left_finger_b_pos = left_gripper_status.finger_b_status.position;
  auto const& left_finger_c_pos = left_gripper_status.finger_c_status.position;
  auto const& left_scissor_pos = left_gripper_status.scissor_status.position;
  auto const& left_finger_a_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(left_finger_a_pos);
  auto const& left_finger_b_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(left_finger_b_pos);
  auto const& left_finger_c_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(left_finger_c_pos);
  hw_states_position_[14] = left_finger_a_thetas[0];
  hw_states_position_[15] = left_finger_a_thetas[1];
  hw_states_position_[16] = left_finger_a_thetas[2];
  hw_states_position_[17] = left_scissor_pos;
  hw_states_position_[18] = left_finger_b_thetas[0];
  hw_states_position_[19] = left_finger_b_thetas[1];
  hw_states_position_[20] = left_finger_b_thetas[2];
  hw_states_position_[21] = -left_scissor_pos;
  hw_states_position_[22] = left_finger_c_thetas[0];
  hw_states_position_[23] = left_finger_c_thetas[1];
  hw_states_position_[24] = left_finger_c_thetas[2];

  auto const& right_gripper_status = right_gripper_status_listener_->getLatestMessage();
  auto const& right_finger_a_pos = right_gripper_status.finger_a_status.position;
  auto const& right_finger_b_pos = right_gripper_status.finger_b_status.position;
  auto const& right_finger_c_pos = right_gripper_status.finger_c_status.position;
  auto const& right_scissor_pos = right_gripper_status.scissor_status.position;
  auto const& right_finger_a_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(right_finger_a_pos);
  auto const& right_finger_b_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(right_finger_b_pos);
  auto const& right_finger_c_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(right_finger_c_pos);
  hw_states_position_[23] = right_finger_a_thetas[0];
  hw_states_position_[24] = right_finger_a_thetas[1];
  hw_states_position_[25] = right_finger_a_thetas[2];
  hw_states_position_[26] = right_scissor_pos;
  hw_states_position_[27] = right_finger_b_thetas[0];
  hw_states_position_[28] = right_finger_b_thetas[1];
  hw_states_position_[29] = right_finger_b_thetas[2];
  hw_states_position_[30] = -right_scissor_pos;
  hw_states_position_[31] = right_finger_c_thetas[0];
  hw_states_position_[32] = right_finger_c_thetas[1];
  hw_states_position_[33] = right_finger_c_thetas[2];

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VictorHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/) {
  // convert hw_pos_cmds_ to LCM messages and publish
  auto const any_nan =
      std::any_of(hw_pos_cmds_.begin(), hw_pos_cmds_.end(), [](auto const& cmd) { return std::isnan(cmd); });
  if (any_nan) {
    return hardware_interface::return_type::OK;
  }

  auto const now = std::chrono::system_clock::now();
  auto const now_tp = std::chrono::time_point_cast<std::chrono::seconds>(now);
  std::chrono::duration<double> const now_dur_seconds = now_tp.time_since_epoch();
  auto const now_seconds = now_dur_seconds.count();

  auto const& left_control_mode = left_control_mode_listener_->getLatestMessage();
  auto const& right_control_mode = right_control_mode_listener_->getLatestMessage();

  victor_lcm_interface::motion_command left_motion_cmd{};
  left_motion_cmd.timestamp = now_seconds;
  left_motion_cmd.control_mode = left_control_mode.control_mode;
  left_motion_cmd.joint_position.joint_1 = hw_pos_cmds_[0];
  left_motion_cmd.joint_position.joint_2 = hw_pos_cmds_[1];
  left_motion_cmd.joint_position.joint_3 = hw_pos_cmds_[2];
  left_motion_cmd.joint_position.joint_4 = hw_pos_cmds_[3];
  left_motion_cmd.joint_position.joint_5 = hw_pos_cmds_[4];
  left_motion_cmd.joint_position.joint_6 = hw_pos_cmds_[5];
  left_motion_cmd.joint_position.joint_7 = hw_pos_cmds_[6];

  victor_lcm_interface::motion_command right_motion_cmd{};
  right_motion_cmd.timestamp = now_seconds;
  right_motion_cmd.control_mode = right_control_mode.control_mode;
  right_motion_cmd.joint_position.joint_1 = hw_pos_cmds_[7];
  right_motion_cmd.joint_position.joint_2 = hw_pos_cmds_[8];
  right_motion_cmd.joint_position.joint_3 = hw_pos_cmds_[9];
  right_motion_cmd.joint_position.joint_4 = hw_pos_cmds_[10];
  right_motion_cmd.joint_position.joint_5 = hw_pos_cmds_[11];
  right_motion_cmd.joint_position.joint_6 = hw_pos_cmds_[12];
  right_motion_cmd.joint_position.joint_7 = hw_pos_cmds_[13];

  // Sending anything other than zero causes an error. Velocity should only be controlled by the control_mode params
  left_motion_cmd.joint_velocity.joint_1 = 0.;
  left_motion_cmd.joint_velocity.joint_2 = 0.;
  left_motion_cmd.joint_velocity.joint_3 = 0.;
  left_motion_cmd.joint_velocity.joint_4 = 0.;
  left_motion_cmd.joint_velocity.joint_5 = 0.;
  left_motion_cmd.joint_velocity.joint_6 = 0.;
  left_motion_cmd.joint_velocity.joint_7 = 0.;
  right_motion_cmd.joint_velocity.joint_1 = 0.;
  right_motion_cmd.joint_velocity.joint_2 = 0.;
  right_motion_cmd.joint_velocity.joint_3 = 0.;
  right_motion_cmd.joint_velocity.joint_4 = 0.;
  right_motion_cmd.joint_velocity.joint_5 = 0.;
  right_motion_cmd.joint_velocity.joint_6 = 0.;
  right_motion_cmd.joint_velocity.joint_7 = 0.;

  // print the hw_pos and hw_vel cmds lists
  // RCLCPP_INFO_STREAM(logger, "left cmd: " << left_motion_cmd.joint_position.joint_1 << " " <<
  // left_motion_cmd.joint_position.joint_2 << " " << left_motion_cmd.joint_position.joint_3 << " " <<
  // left_motion_cmd.joint_position.joint_4 << " " << left_motion_cmd.joint_position.joint_5 << " " <<
  // left_motion_cmd.joint_position.joint_6 << " " << left_motion_cmd.joint_position.joint_7);
  // RCLCPP_INFO_STREAM(logger, "right cmd: " << right_motion_cmd.joint_position.joint_1 << " " <<
  // right_motion_cmd.joint_position.joint_2 << " " << right_motion_cmd.joint_position.joint_3 << " " <<
  // right_motion_cmd.joint_position.joint_4 << " " << right_motion_cmd.joint_position.joint_5 << " " <<
  // right_motion_cmd.joint_position.joint_6 << " " << right_motion_cmd.joint_position.joint_7);

  left_send_lcm_ptr_->publish(DEFAULT_MOTION_COMMAND_CHANNEL, &left_motion_cmd);
  right_send_lcm_ptr_->publish(DEFAULT_MOTION_COMMAND_CHANNEL, &right_motion_cmd);

  return hardware_interface::return_type::OK;
}

}  // namespace victor_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(victor_hardware::VictorHardwareInterface, hardware_interface::SystemInterface)