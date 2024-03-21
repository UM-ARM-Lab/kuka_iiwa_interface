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

  RCLCPP_INFO(logger, "===================================================================================");
  RCLCPP_INFO(logger, "Please start the LCMRobotInterface application on BOTH pendants!");
  RCLCPP_INFO(logger, "===================================================================================");

  node_ = std::make_shared<rclcpp::Node>("victor_hardware_interface_node");

  setter_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  executor_ = std::make_shared<AsyncExecutor>();
  executor_->add_node(node_);

  // Some of these arrays will be bigger than they need to be, since not all joints are actuated (e.g. finger joints)
  RCLCPP_INFO_STREAM(logger, "Found " << info_.joints.size() << " joints");

  hw_cmds_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_external_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_cmd_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_external_torque_sensor_.resize(info_.joints.size(), 0);

  // NOTE: changing these values here requires corresponding changes in the LCMRobotInterface application
  //  and also the victor_lcm_bridge launch file.
  std::string const left_recv_provider = "udp://10.10.10.169:30002";
  std::string const right_recv_provider = "udp://10.10.10.169:30001";
  std::string const left_send_provider = "udp://10.10.10.12:30000";
  std::string const right_send_provider = "udp://10.10.10.11:30000";

  left.on_init(executor_, node_, left_send_provider, left_recv_provider);
  right.on_init(executor_, node_, right_send_provider, right_recv_provider);

  //  // Example usage of DataTamer
  //  sink_ = std::make_shared<DataTamer::MCAPSink>("/home/armlab/victor_hw_if.mcap");
  //
  //  // Create a channel and attach a sink. A channel can have multiple sinks
  //  channel_ = DataTamer::LogChannel::create("hw_pos_cmds");
  //  channel_->addDataSink(sink_);

  //  value_ = channel_->registerValue("values", &hw_cmds_position_);

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

  // export state interfaces for force torque sensor
  state_interfaces.emplace_back("left_force_torque_sensor", "force.x", &left.hw_ft_[0]);
  state_interfaces.emplace_back("left_force_torque_sensor", "force.y", &left.hw_ft_[1]);
  state_interfaces.emplace_back("left_force_torque_sensor", "force.z", &left.hw_ft_[2]);
  state_interfaces.emplace_back("left_force_torque_sensor", "torque.x", &left.hw_ft_[3]);
  state_interfaces.emplace_back("left_force_torque_sensor", "torque.y", &left.hw_ft_[4]);
  state_interfaces.emplace_back("left_force_torque_sensor", "torque.z", &left.hw_ft_[5]);
  state_interfaces.emplace_back("right_force_torque_sensor", "force.x", &right.hw_ft_[0]);
  state_interfaces.emplace_back("right_force_torque_sensor", "force.y", &right.hw_ft_[1]);
  state_interfaces.emplace_back("right_force_torque_sensor", "force.z", &right.hw_ft_[2]);
  state_interfaces.emplace_back("right_force_torque_sensor", "torque.x", &right.hw_ft_[3]);
  state_interfaces.emplace_back("right_force_torque_sensor", "torque.y", &right.hw_ft_[4]);
  state_interfaces.emplace_back("right_force_torque_sensor", "torque.z", &right.hw_ft_[5]);

  // export state interfaces for motion status,

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
      command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &hw_cmds_position_[i]);
    }
  }

  // Cartesian pose interfaces
  command_interfaces.emplace_back("left", "cartesian_pose/xt", &left.hw_cmd_cartesian_pose_.position.x);
  command_interfaces.emplace_back("left", "cartesian_pose/yt", &left.hw_cmd_cartesian_pose_.position.y);
  command_interfaces.emplace_back("left", "cartesian_pose/zt", &left.hw_cmd_cartesian_pose_.position.z);
  command_interfaces.emplace_back("left", "cartesian_pose/wr", &left.hw_cmd_cartesian_pose_.orientation.w);
  command_interfaces.emplace_back("left", "cartesian_pose/xr", &left.hw_cmd_cartesian_pose_.orientation.x);
  command_interfaces.emplace_back("left", "cartesian_pose/yr", &left.hw_cmd_cartesian_pose_.orientation.y);
  command_interfaces.emplace_back("left", "cartesian_pose/zr", &left.hw_cmd_cartesian_pose_.orientation.z);

  command_interfaces.emplace_back("right", "cartesian_pose/xt", &right.hw_cmd_cartesian_pose_.position.x);
  command_interfaces.emplace_back("right", "cartesian_pose/yt", &right.hw_cmd_cartesian_pose_.position.y);
  command_interfaces.emplace_back("right", "cartesian_pose/zt", &right.hw_cmd_cartesian_pose_.position.z);
  command_interfaces.emplace_back("right", "cartesian_pose/wr", &right.hw_cmd_cartesian_pose_.orientation.w);
  command_interfaces.emplace_back("right", "cartesian_pose/xr", &right.hw_cmd_cartesian_pose_.orientation.x);
  command_interfaces.emplace_back("right", "cartesian_pose/yr", &right.hw_cmd_cartesian_pose_.orientation.y);
  command_interfaces.emplace_back("right", "cartesian_pose/zr", &right.hw_cmd_cartesian_pose_.orientation.z);

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
    left.recv_lcm_ptr_->handleTimeout(1000);
    right.recv_lcm_ptr_->handleTimeout(1000);
  }
}

// ------------------------------------------------------------------------------------------
hardware_interface::return_type VictorHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) {
  if (get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return hardware_interface::return_type::OK;
  }

  if (!left.motion_status_listener_->hasLatestMessage() || !right.motion_status_listener_->hasLatestMessage() ||
      !left.gripper_status_listener_->hasLatestMessage() || !right.gripper_status_listener_->hasLatestMessage()) {
    return hardware_interface::return_type::OK;
  }
  auto const& left_motion_status = left.motion_status_listener_->getLatestMessage();
  auto const& right_motion_status = right.motion_status_listener_->getLatestMessage();

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

  hw_states_external_effort_[0] = left_motion_status.measured_joint_torque.joint_1;
  hw_states_external_effort_[1] = left_motion_status.measured_joint_torque.joint_2;
  hw_states_external_effort_[2] = left_motion_status.measured_joint_torque.joint_3;
  hw_states_external_effort_[3] = left_motion_status.measured_joint_torque.joint_4;
  hw_states_external_effort_[4] = left_motion_status.measured_joint_torque.joint_5;
  hw_states_external_effort_[5] = left_motion_status.measured_joint_torque.joint_6;
  hw_states_external_effort_[6] = left_motion_status.measured_joint_torque.joint_7;
  hw_states_external_effort_[7] = right_motion_status.measured_joint_torque.joint_1;
  hw_states_external_effort_[8] = right_motion_status.measured_joint_torque.joint_2;
  hw_states_external_effort_[9] = right_motion_status.measured_joint_torque.joint_3;
  hw_states_external_effort_[10] = right_motion_status.measured_joint_torque.joint_4;
  hw_states_external_effort_[11] = right_motion_status.measured_joint_torque.joint_5;
  hw_states_external_effort_[12] = right_motion_status.measured_joint_torque.joint_6;
  hw_states_external_effort_[13] = right_motion_status.measured_joint_torque.joint_7;

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

  // Copy the estimated external force torque readings from the motion status into the state interface
  left.hw_ft_[0] = left_motion_status.estimated_external_wrench.x;
  left.hw_ft_[1] = left_motion_status.estimated_external_wrench.y;
  left.hw_ft_[2] = left_motion_status.estimated_external_wrench.z;
  left.hw_ft_[3] = left_motion_status.estimated_external_wrench.a;
  left.hw_ft_[4] = left_motion_status.estimated_external_wrench.b;
  left.hw_ft_[5] = left_motion_status.estimated_external_wrench.c;
  right.hw_ft_[0] = right_motion_status.estimated_external_wrench.x;
  right.hw_ft_[1] = right_motion_status.estimated_external_wrench.y;
  right.hw_ft_[2] = right_motion_status.estimated_external_wrench.z;
  right.hw_ft_[3] = right_motion_status.estimated_external_wrench.a;
  right.hw_ft_[4] = right_motion_status.estimated_external_wrench.b;
  right.hw_ft_[5] = right_motion_status.estimated_external_wrench.c;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VictorHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/) {
  if (get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return hardware_interface::return_type::OK;
  }

  auto const& has_left_motion_status = left.motion_status_listener_->hasLatestMessage();
  auto const& has_right_motion_status = right.motion_status_listener_->hasLatestMessage();
  auto const& has_left_control_mode = left.control_mode_listener_->hasLatestMessage();
  auto const& has_right_control_mode = right.control_mode_listener_->hasLatestMessage();

  if (!has_left_motion_status || !has_right_motion_status || !has_left_control_mode || !has_right_control_mode) {
    return hardware_interface::return_type::OK;
  }

  auto const& left_motion_status = left.motion_status_listener_->getLatestMessage();
  auto const& right_motion_status = right.motion_status_listener_->getLatestMessage();

  std::array<double, 14> current_commanded_positions;
  current_commanded_positions[0] = left_motion_status.commanded_joint_position.joint_1;
  current_commanded_positions[1] = left_motion_status.commanded_joint_position.joint_2;
  current_commanded_positions[2] = left_motion_status.commanded_joint_position.joint_3;
  current_commanded_positions[3] = left_motion_status.commanded_joint_position.joint_4;
  current_commanded_positions[4] = left_motion_status.commanded_joint_position.joint_5;
  current_commanded_positions[5] = left_motion_status.commanded_joint_position.joint_6;
  current_commanded_positions[6] = left_motion_status.commanded_joint_position.joint_7;
  current_commanded_positions[7] = right_motion_status.commanded_joint_position.joint_1;
  current_commanded_positions[8] = right_motion_status.commanded_joint_position.joint_2;
  current_commanded_positions[9] = right_motion_status.commanded_joint_position.joint_3;
  current_commanded_positions[10] = right_motion_status.commanded_joint_position.joint_4;
  current_commanded_positions[11] = right_motion_status.commanded_joint_position.joint_5;
  current_commanded_positions[12] = right_motion_status.commanded_joint_position.joint_6;
  current_commanded_positions[13] = right_motion_status.commanded_joint_position.joint_7;

  rclcpp::Clock clock;
  for (int i = 0; i < 14; i++) {
    if (std::isnan(hw_cmds_position_[i])) {
      RCLCPP_WARN_THROTTLE(logger, clock, 500,
                           "Joint %d is NaN, using current commanded position according to the robot", i);
      hw_cmds_position_[i] = current_commanded_positions[i];
    }
  }

  // convert hw_cmds_position_ to LCM messages and publish
  auto const now = std::chrono::system_clock::now();
  auto const now_tp = std::chrono::time_point_cast<std::chrono::seconds>(now);
  std::chrono::duration<double> const now_dur_seconds = now_tp.time_since_epoch();
  auto const now_seconds = now_dur_seconds.count();

  auto const& left_control_mode = left.control_mode_listener_->getLatestMessage();
  auto const& right_control_mode = right.control_mode_listener_->getLatestMessage();

  victor_lcm_interface::motion_command left_motion_cmd{};
  left_motion_cmd.timestamp = now_seconds;
  left_motion_cmd.control_mode = left_control_mode.control_mode;
  left_motion_cmd.joint_position.joint_1 = hw_cmds_position_[0];
  left_motion_cmd.joint_position.joint_2 = hw_cmds_position_[1];
  left_motion_cmd.joint_position.joint_3 = hw_cmds_position_[2];
  left_motion_cmd.joint_position.joint_4 = hw_cmds_position_[3];
  left_motion_cmd.joint_position.joint_5 = hw_cmds_position_[4];
  left_motion_cmd.joint_position.joint_6 = hw_cmds_position_[5];
  left_motion_cmd.joint_position.joint_7 = hw_cmds_position_[6];
  // left_motion_cmd.cartesian_pose // TODO!!!

  victor_lcm_interface::motion_command right_motion_cmd{};
  right_motion_cmd.timestamp = now_seconds;
  right_motion_cmd.control_mode = right_control_mode.control_mode;
  right_motion_cmd.joint_position.joint_1 = hw_cmds_position_[7];
  right_motion_cmd.joint_position.joint_2 = hw_cmds_position_[8];
  right_motion_cmd.joint_position.joint_3 = hw_cmds_position_[9];
  right_motion_cmd.joint_position.joint_4 = hw_cmds_position_[10];
  right_motion_cmd.joint_position.joint_5 = hw_cmds_position_[11];
  right_motion_cmd.joint_position.joint_6 = hw_cmds_position_[12];
  right_motion_cmd.joint_position.joint_7 = hw_cmds_position_[13];

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

  left.send_motion_command(left_motion_cmd);
  right.send_motion_command(right_motion_cmd);

  //  channel_->takeSnapshot();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VictorHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) {
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VictorHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) {
  RCLCPP_WARN_STREAM(logger, "performing switch!");
  return hardware_interface::return_type::OK;
}

}  // namespace victor_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(victor_hardware::VictorHardwareInterface, hardware_interface::SystemInterface)
