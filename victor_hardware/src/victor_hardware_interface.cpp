#include <span>
#include <victor_hardware/constants.hpp>
#include <victor_hardware/victor_hardware_interface.hpp>

#include "rclcpp/rclcpp.hpp"

auto logger = rclcpp::get_logger("VictorHardwareInterface");

namespace victor_hardware {
// ------------------------------------------------------------------------------------------
CallbackReturn VictorHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // This controller does all 14 joints, 7 on each arm. The ros2 control interface seems to require
  // that the positions/velocities/etc are all stored as contiguous arrays (i.e std::vector) so all
  // these vectors are of size 14. But of course each arm has its own  interface that only understands
  // 7 joints, so we use std::span to create views into the vectors that only contain the relevant joints,
  // which shouldn't do any copying.
  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_pos_cmds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_external_torque_sensor_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface> VictorHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
  }
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));
  }
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, "external_torque", &hw_states_external_torque_sensor_[i]));
  }

  return state_interfaces;
}

// ------------------------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface> VictorHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    RCLCPP_INFO(logger, "Exporting command interfaces for joint %s", info_.joints[i].name.c_str());
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_pos_cmds_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_vel_cmds_[i]));
  }

  return command_interfaces;
}

// ------------------------------------------------------------------------------------------
CallbackReturn VictorHardwareInterface::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  RCLCPP_INFO(logger, "Starting ...please wait...");

  // NOTE: changing these values here requires corresponding changes in the LCMRobotInterface application
  //  and also the victor_lcm_bridge launch file.
  std::string const left_recv_provider = "udp://10.10.10.108:30002";
  std::string const right_recv_provider = "udp://10.10.10.108:30001";
  std::string const left_send_provider = "udp://10.10.10.12:30000";
  std::string const right_send_provider = "udp://10.10.10.11:30000";

  // set default value for sensor
  for (auto i = 0ul; i < hw_states_external_torque_sensor_.size(); i++) {
    if (std::isnan(hw_states_external_torque_sensor_[i])) {
      hw_states_external_torque_sensor_[i] = 0;
    }
  }

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

  left_recv_lcm_ptr_->subscribe(DEFAULT_MOTION_STATUS_CHANNEL, &VictorHardwareInterface::LeftMotionStatusCallback,
                                this);
  left_recv_lcm_ptr_->subscribe(DEFAULT_CONTROL_MODE_STATUS_CHANNEL,
                                &VictorHardwareInterface::LeftControlModeStatusCallback, this);
  right_recv_lcm_ptr_->subscribe(DEFAULT_MOTION_STATUS_CHANNEL, &VictorHardwareInterface::RightMotionStatusCallback,
                                 this);
  right_recv_lcm_ptr_->subscribe(DEFAULT_CONTROL_MODE_STATUS_CHANNEL,
                                 &VictorHardwareInterface::RightControlModeStatusCallback, this);

  lcm_thread_running_ = true;
  lcm_thread_ = std::thread(&VictorHardwareInterface::LCMThread, this);

  RCLCPP_INFO(logger, "On Activate finished successfully");

  return CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------------------------------
CallbackReturn VictorHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
  // Stop the LCM processing thread
  RCLCPP_INFO(logger, "Stopping LCM processing thread");

  lcm_thread_running_ = false;
  lcm_thread_.join();

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
  victor_lcm_interface::motion_status left_motion_status{};
  victor_lcm_interface::motion_status right_motion_status{};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    left_motion_status = latest_left_motion_status_;
    right_motion_status = latest_right_motion_status_;
  }
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

  hw_states_velocity_[0] = left_motion_status.measured_joint_velocity.joint_1;
  hw_states_velocity_[1] = left_motion_status.measured_joint_velocity.joint_2;
  hw_states_velocity_[2] = left_motion_status.measured_joint_velocity.joint_3;
  hw_states_velocity_[3] = left_motion_status.measured_joint_velocity.joint_4;
  hw_states_velocity_[4] = left_motion_status.measured_joint_velocity.joint_5;
  hw_states_velocity_[5] = left_motion_status.measured_joint_velocity.joint_6;
  hw_states_velocity_[6] = left_motion_status.measured_joint_velocity.joint_7;
  hw_states_velocity_[7] = right_motion_status.measured_joint_velocity.joint_1;
  hw_states_velocity_[8] = right_motion_status.measured_joint_velocity.joint_2;
  hw_states_velocity_[9] = right_motion_status.measured_joint_velocity.joint_3;
  hw_states_velocity_[10] = right_motion_status.measured_joint_velocity.joint_4;
  hw_states_velocity_[11] = right_motion_status.measured_joint_velocity.joint_5;
  hw_states_velocity_[12] = right_motion_status.measured_joint_velocity.joint_6;
  hw_states_velocity_[13] = right_motion_status.measured_joint_velocity.joint_7;

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
  hw_states_external_torque_sensor_[10] = latest_right_motion_status_.estimated_external_torque.joint_4;
  hw_states_external_torque_sensor_[11] = latest_right_motion_status_.estimated_external_torque.joint_5;
  hw_states_external_torque_sensor_[12] = latest_right_motion_status_.estimated_external_torque.joint_6;
  hw_states_external_torque_sensor_[13] = latest_right_motion_status_.estimated_external_torque.joint_7;

  return hardware_interface::return_type::OK;
}
// ------------------------------------------------------------------------------------------
hardware_interface::return_type VictorHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/) {
  // convert hw_pos_cmds_ to LCM messages and publish
  bool isNan = false;
  for (auto i = 0ul; i < hw_pos_cmds_.size(); i++) {
    if (std::isnan(hw_pos_cmds_[i])) {
      return hardware_interface::return_type::ERROR;
    }
  }

  victor_lcm_interface::motion_command left_motion_cmd{};
  left_motion_cmd.control_mode = latest_left_control_mode_;
  left_motion_cmd.joint_position.joint_1 = hw_pos_cmds_[0];
  left_motion_cmd.joint_position.joint_2 = hw_pos_cmds_[1];
  left_motion_cmd.joint_position.joint_3 = hw_pos_cmds_[2];
  left_motion_cmd.joint_position.joint_4 = hw_pos_cmds_[3];
  left_motion_cmd.joint_position.joint_5 = hw_pos_cmds_[4];
  left_motion_cmd.joint_position.joint_6 = hw_pos_cmds_[5];
  left_motion_cmd.joint_position.joint_7 = hw_pos_cmds_[6];
  left_motion_cmd.joint_velocity.joint_1 = hw_vel_cmds_[0];
  left_motion_cmd.joint_velocity.joint_2 = hw_vel_cmds_[1];
  left_motion_cmd.joint_velocity.joint_3 = hw_vel_cmds_[2];
  left_motion_cmd.joint_velocity.joint_4 = hw_vel_cmds_[3];
  left_motion_cmd.joint_velocity.joint_5 = hw_vel_cmds_[4];
  left_motion_cmd.joint_velocity.joint_6 = hw_vel_cmds_[5];
  left_motion_cmd.joint_velocity.joint_7 = hw_vel_cmds_[6];

  victor_lcm_interface::motion_command right_motion_cmd{};
  right_motion_cmd.control_mode = latest_right_control_mode_;
  right_motion_cmd.joint_position.joint_1 = hw_pos_cmds_[7];
  right_motion_cmd.joint_position.joint_2 = hw_pos_cmds_[8];
  right_motion_cmd.joint_position.joint_3 = hw_pos_cmds_[9];
  right_motion_cmd.joint_position.joint_4 = hw_pos_cmds_[10];
  right_motion_cmd.joint_position.joint_5 = hw_pos_cmds_[11];
  right_motion_cmd.joint_position.joint_6 = hw_pos_cmds_[12];
  right_motion_cmd.joint_position.joint_7 = hw_pos_cmds_[13];
  right_motion_cmd.joint_velocity.joint_1 = hw_vel_cmds_[7];
  right_motion_cmd.joint_velocity.joint_2 = hw_vel_cmds_[8];
  right_motion_cmd.joint_velocity.joint_3 = hw_vel_cmds_[9];
  right_motion_cmd.joint_velocity.joint_4 = hw_vel_cmds_[10];
  right_motion_cmd.joint_velocity.joint_5 = hw_vel_cmds_[11];
  right_motion_cmd.joint_velocity.joint_6 = hw_vel_cmds_[12];
  right_motion_cmd.joint_velocity.joint_7 = hw_vel_cmds_[13];

  left_send_lcm_ptr_->publish(DEFAULT_MOTION_COMMAND_CHANNEL, &left_motion_cmd);
  right_send_lcm_ptr_->publish(DEFAULT_MOTION_COMMAND_CHANNEL, &right_motion_cmd);

  return hardware_interface::return_type::OK;
}

void VictorHardwareInterface::LeftMotionStatusCallback(const lcm::ReceiveBuffer* /*buffer*/,
                                                       const std::string& /*channel*/,
                                                       const victor_lcm_interface::motion_status *motion_status) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_left_motion_status_ = *motion_status;
}

void VictorHardwareInterface::LeftControlModeStatusCallback(const lcm::ReceiveBuffer* /*buffer*/,
                                                            const std::string& /*channel*/,
                                                            const victor_lcm_interface::control_mode_parameters *control_mode_parameters) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_left_control_mode_ = control_mode_parameters->control_mode;
}

void VictorHardwareInterface::RightMotionStatusCallback(const lcm::ReceiveBuffer* /*buffer*/,
                                                        const std::string& /*channel*/,
                                                        const victor_lcm_interface::motion_status *motion_status) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_right_motion_status_ = *motion_status;
}

void VictorHardwareInterface::RightControlModeStatusCallback(const lcm::ReceiveBuffer* /*buffer*/,
                                                             const std::string& /*channel*/,
                                                             const victor_lcm_interface::control_mode_parameters *control_mode_parameters) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_right_control_mode_ = control_mode_parameters->control_mode;
}

}  // namespace victor_hardware
// ---------------------------------------------------------------------------
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(victor_hardware::VictorHardwareInterface, hardware_interface::SystemInterface)