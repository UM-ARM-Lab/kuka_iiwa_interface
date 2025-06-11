#include "victor_sim_hardware/side.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <victor_sim_hardware/constants.hpp>

using namespace std::placeholders;

namespace victor_sim_hardware {

Side::Side(std::string const& name) : side_name_(name), logger_(rclcpp::get_logger("VictorSimHardwareInterface.Side." + name)) {
  hw_state_ft_.fill(0.0);
  
  // Initialize motion command with default values
  motion_cmd_.commanded_joint_position.joint_1 = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.commanded_joint_position.joint_2 = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.commanded_joint_position.joint_3 = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.commanded_joint_position.joint_4 = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.commanded_joint_position.joint_5 = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.commanded_joint_position.joint_6 = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.commanded_joint_position.joint_7 = std::numeric_limits<double>::quiet_NaN();

  motion_cmd_.measured_joint_velocity.joint_1 = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.measured_joint_velocity.joint_2 = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.measured_joint_velocity.joint_3 = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.measured_joint_velocity.joint_4 = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.measured_joint_velocity.joint_5 = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.measured_joint_velocity.joint_6 = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.measured_joint_velocity.joint_7 = std::numeric_limits<double>::quiet_NaN();
  
  // Initialize cartesian pose to identity
  motion_cmd_.commanded_cartesian_pose.position.x = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.commanded_cartesian_pose.position.y = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.commanded_cartesian_pose.position.z = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.commanded_cartesian_pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.commanded_cartesian_pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.commanded_cartesian_pose.orientation.y = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.commanded_cartesian_pose.orientation.z = std::numeric_limits<double>::quiet_NaN();
}

void Side::add_state_interfaces(std::vector<hardware_interface::StateInterface>& state_interfaces) {
  state_interfaces.emplace_back(side_name_ + "_force_torque_sensor", "force.x", &hw_state_ft_[0]);
  state_interfaces.emplace_back(side_name_ + "_force_torque_sensor", "force.y", &hw_state_ft_[1]);
  state_interfaces.emplace_back(side_name_ + "_force_torque_sensor", "force.z", &hw_state_ft_[2]);
  state_interfaces.emplace_back(side_name_ + "_force_torque_sensor", "torque.x", &hw_state_ft_[3]);
  state_interfaces.emplace_back(side_name_ + "_force_torque_sensor", "torque.y", &hw_state_ft_[4]);
  state_interfaces.emplace_back(side_name_ + "_force_torque_sensor", "torque.z", &hw_state_ft_[5]);

  state_interfaces.emplace_back(side_name_, MEASURED_XT_STATE_INTERFACE, &hw_state_cartesian_pose_.position.x);
  state_interfaces.emplace_back(side_name_, MEASURED_YT_STATE_INTERFACE, &hw_state_cartesian_pose_.position.y);
  state_interfaces.emplace_back(side_name_, MEASURED_ZT_STATE_INTERFACE, &hw_state_cartesian_pose_.position.z);
  state_interfaces.emplace_back(side_name_, MEASURED_WR_STATE_INTERFACE, &hw_state_cartesian_pose_.orientation.w);
  state_interfaces.emplace_back(side_name_, MEASURED_XR_STATE_INTERFACE, &hw_state_cartesian_pose_.orientation.x);
  state_interfaces.emplace_back(side_name_, MEASURED_YR_STATE_INTERFACE, &hw_state_cartesian_pose_.orientation.y);
  state_interfaces.emplace_back(side_name_, MEASURED_ZR_STATE_INTERFACE, &hw_state_cartesian_pose_.orientation.z);

  state_interfaces.emplace_back(side_name_, COMMANDED_XT_STATE_INTERFACE, &hw_state_cmd_cartesian_pose_.position.x);
  state_interfaces.emplace_back(side_name_, COMMANDED_YT_STATE_INTERFACE, &hw_state_cmd_cartesian_pose_.position.y);
  state_interfaces.emplace_back(side_name_, COMMANDED_ZT_STATE_INTERFACE, &hw_state_cmd_cartesian_pose_.position.z);
  state_interfaces.emplace_back(side_name_, COMMANDED_WR_STATE_INTERFACE, &hw_state_cmd_cartesian_pose_.orientation.w);
  state_interfaces.emplace_back(side_name_, COMMANDED_XR_STATE_INTERFACE, &hw_state_cmd_cartesian_pose_.orientation.x);
  state_interfaces.emplace_back(side_name_, COMMANDED_YR_STATE_INTERFACE, &hw_state_cmd_cartesian_pose_.orientation.y);
  state_interfaces.emplace_back(side_name_, COMMANDED_ZR_STATE_INTERFACE, &hw_state_cmd_cartesian_pose_.orientation.z);
}

void Side::add_command_interfaces(hardware_interface::HardwareInfo const& info,
                                  std::vector<hardware_interface::CommandInterface>& command_interfaces) {
  // Cartesian pose interfaces
  command_interfaces.emplace_back(side_name_, CARTESIAN_XT_INTERFACE, &motion_cmd_.commanded_cartesian_pose.position.x);
  command_interfaces.emplace_back(side_name_, CARTESIAN_YT_INTERFACE, &motion_cmd_.commanded_cartesian_pose.position.y);
  command_interfaces.emplace_back(side_name_, CARTESIAN_ZT_INTERFACE, &motion_cmd_.commanded_cartesian_pose.position.z);
  command_interfaces.emplace_back(side_name_, CARTESIAN_WR_INTERFACE, &motion_cmd_.commanded_cartesian_pose.orientation.w);
  command_interfaces.emplace_back(side_name_, CARTESIAN_XR_INTERFACE, &motion_cmd_.commanded_cartesian_pose.orientation.x);
  command_interfaces.emplace_back(side_name_, CARTESIAN_YR_INTERFACE, &motion_cmd_.commanded_cartesian_pose.orientation.y);
  command_interfaces.emplace_back(side_name_, CARTESIAN_ZR_INTERFACE, &motion_cmd_.commanded_cartesian_pose.orientation.z);

  // The interface names should be consistent with ros2_controllers.yaml and URDF
  auto const& prefix = "victor_" + side_name_ + "_arm_";
  command_interfaces.emplace_back(prefix + "joint_1", hardware_interface::HW_IF_POSITION,
                                  &motion_cmd_.commanded_joint_position.joint_1);
  command_interfaces.emplace_back(prefix + "joint_2", hardware_interface::HW_IF_POSITION,
                                  &motion_cmd_.commanded_joint_position.joint_2);
  command_interfaces.emplace_back(prefix + "joint_3", hardware_interface::HW_IF_POSITION,
                                  &motion_cmd_.commanded_joint_position.joint_3);
  command_interfaces.emplace_back(prefix + "joint_4", hardware_interface::HW_IF_POSITION,
                                  &motion_cmd_.commanded_joint_position.joint_4);
  command_interfaces.emplace_back(prefix + "joint_5", hardware_interface::HW_IF_POSITION,
                                  &motion_cmd_.commanded_joint_position.joint_5);
  command_interfaces.emplace_back(prefix + "joint_6", hardware_interface::HW_IF_POSITION,
                                  &motion_cmd_.commanded_joint_position.joint_6);
  command_interfaces.emplace_back(prefix + "joint_7", hardware_interface::HW_IF_POSITION,
                                  &motion_cmd_.commanded_joint_position.joint_7);

  // Command interfaces to represent the control modes
  command_interfaces.emplace_back(side_name_, JOINT_POSITION_INTERFACE, &hw_cmd_joint_position_control_mode_);
  command_interfaces.emplace_back(side_name_, JOINT_IMPEDANCE_INTERFACE, &hw_cmd_joint_impedance_control_mode_);
  command_interfaces.emplace_back(side_name_, CARTESIAN_POSE_INTERFACE, &hw_cmd_cartesian_pose_control_mode_);
  command_interfaces.emplace_back(side_name_, CARTESIAN_IMPEDANCE_INTERFACE, &hw_cmd_cartesian_impedance_control_mode_);
}

CallbackReturn Side::on_init(std::shared_ptr<rclcpp::Node> const& node) {
  RCLCPP_INFO(logger_, "Initializing Side %s...", side_name_.c_str());

  // Initialize state
  hw_state_ft_.fill(0);
  has_active_controller_ = false;
  latest_control_mode_ = victor_sim_hardware::control_mode::JOINT_POSITION;

  // Create callback groups for thread safety
  getter_callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  setter_callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto setter_options = rclcpp::SubscriptionOptions();
  setter_options.callback_group = setter_callback_group_;

  auto getter_options = rclcpp::SubscriptionOptions();
  getter_options.callback_group = getter_callback_group_;

  // Standard victor API (compatible with existing controllers)
  auto const ns = "/victor/" + side_name_ + "_arm/";
  motion_status_pub_ = node->create_publisher<msg::MotionStatus>(ns + DEFAULT_MOTION_STATUS_TOPIC, 10);
  control_mode_params_pub_ =
      node->create_publisher<msg::ControlModeParameters>(ns + DEFAULT_CONTROL_MODE_PARAMETERS_TOPIC, 10);
  gripper_status_pub_ = node->create_publisher<msg::Robotiq3FingerStatus>(ns + DEFAULT_GRIPPER_STATUS_TOPIC, 10);
  gripper_command_sub_ = node->create_subscription<msg::Robotiq3FingerCommand>(
      ns + DEFAULT_GRIPPER_COMMAND_TOPIC, 10,
      std::bind(&Side::gripperCommandROSCallback, this, std::placeholders::_1), setter_options);

  // Simulator bridge communication (NEW - for Python simulator)
  auto const sim_ns = "/victor_sim_bridge/" + side_name_ + "/";
  sim_motion_command_pub_ = node->create_publisher<msg::MotionStatus>(sim_ns + "motion_command", 1);
  sim_gripper_command_pub_ = node->create_publisher<msg::Robotiq3FingerCommand>(sim_ns + "gripper_command", 1);
  
  sim_motion_status_sub_ = node->create_subscription<msg::MotionStatus>(
      sim_ns + "motion_status", 1,
      std::bind(&Side::motionStatusSimCallback, this, std::placeholders::_1), getter_options);
      
  sim_gripper_status_sub_ = node->create_subscription<msg::Robotiq3FingerStatus>(
      sim_ns + "gripper_status", 1,
      std::bind(&Side::gripperStatusSimCallback, this, std::placeholders::_1), getter_options);

  // Initialize control mode client with callback to publish control mode parameters
  control_mode_client_ = std::make_shared<SimControlModeClient>(
      node, std::bind(&Side::controlModeParametersCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Side %s initialized successfully", side_name_.c_str());
  return CallbackReturn::SUCCESS;
}

void Side::gripperCommandROSCallback(const msg::Robotiq3FingerCommand& command) {
  // RCLCPP_DEBUG(logger_, "Received gripper command for side %s", side_name_.c_str());
  
  // Forward gripper command to simulator
  sim_gripper_command_pub_->publish(command);
}

void Side::motionStatusSimCallback(const msg::MotionStatus& status) {
  // RCLCPP_INFO(logger_, "Received motion status for side %s: joint_1=%.3f", 
  //             side_name_.c_str(), status.measured_joint_position.joint_1);
  
  // Store latest status from simulator
  latest_motion_status_ = status;
  has_motion_status_ = true;
  
  // Update internal state from simulator feedback
  read_motion_status(status);
  
  // Republish on standard victor API topic for controllers
  publish_motion_status(status);
}

void Side::gripperStatusSimCallback(const msg::Robotiq3FingerStatus& status) {
  // Store latest gripper status from simulator
  latest_gripper_status_ = status;
  has_gripper_status_ = true;
  
  // Republish gripper status on standard victor API topic
  publish_gripper_status(status);
}

void Side::controlModeParametersCallback(const msg::ControlModeParameters& params) {
  RCLCPP_INFO(logger_, "Publishing control mode parameters for side %s, mode: %d", 
              side_name_.c_str(), params.control_mode.mode);
  
  // Publish control mode parameters on the standard victor API topic
  control_mode_params_pub_->publish(params);
}

void Side::read_motion_status(const msg::MotionStatus& status) {
  // Store latest status
  hw_state_motion_status_ = status;

  // Copy the estimated external force torque readings from the motion status into the state interface
  hw_state_ft_[0] = status.estimated_external_wrench.x;
  hw_state_ft_[1] = status.estimated_external_wrench.y;
  hw_state_ft_[2] = status.estimated_external_wrench.z;
  hw_state_ft_[3] = status.estimated_external_wrench.a;
  hw_state_ft_[4] = status.estimated_external_wrench.b;
  hw_state_ft_[5] = status.estimated_external_wrench.c;

  hw_state_cartesian_pose_ = status.measured_cartesian_pose;
  hw_state_cmd_cartesian_pose_ = status.commanded_cartesian_pose;
}

void Side::publish_motion_status(const msg::MotionStatus& msg) {
  motion_status_pub_->publish(msg);
}

void Side::publish_gripper_status(const msg::Robotiq3FingerStatus& msg) {
  gripper_status_pub_->publish(msg);
}

hardware_interface::return_type Side::send_motion_command() {
  rclcpp::Clock clock;
  if (!has_active_controller_) {
    RCLCPP_INFO_THROTTLE(logger_, clock, 5000, "No active controller yet, not sending motion command");
    return hardware_interface::return_type::OK;
  }

  // Create motion command message from current command values
  auto motion_cmd = createMotionCommandMessage();
  
  // Validate the command
  if (!validateMotionCommand()) {
    RCLCPP_WARN_STREAM_THROTTLE(logger_, clock, 5000, "Invalid motion command, not sending");
    return hardware_interface::return_type::ERROR;
  }

  // Send to simulator via ROS
  sim_motion_command_pub_->publish(motion_cmd);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Side::perform_command_mode_switch(const std::vector<std::string>& start_interfaces) {
  RCLCPP_INFO(logger_, "Performing command mode switch for side %s", side_name_.c_str());
  
  // Follow the same pattern as real hardware with lambda function
  auto update_control_mode_and_active_controller = [&](auto mode) {
      latest_control_mode_ = mode;
      has_active_controller_ = control_mode_client_->updateControlMode(latest_control_mode_);
      reset_motion_cmd_to_current_measured();
      return has_active_controller_;
  };

  bool success = true;
  for (auto const& interface : start_interfaces) {
    if (interface == side_name_ + "/" + JOINT_POSITION_INTERFACE) {
      success = update_control_mode_and_active_controller(victor_sim_hardware::control_mode::JOINT_POSITION);
    } else if (interface == side_name_ + "/" + JOINT_IMPEDANCE_INTERFACE) {
      success = update_control_mode_and_active_controller(victor_sim_hardware::control_mode::JOINT_IMPEDANCE);
    } else if (interface == side_name_ + "/" + CARTESIAN_POSE_INTERFACE) {
      success = update_control_mode_and_active_controller(victor_sim_hardware::control_mode::CARTESIAN_POSE);
    } else if (interface == side_name_ + "/" + CARTESIAN_IMPEDANCE_INTERFACE) {
      success = update_control_mode_and_active_controller(victor_sim_hardware::control_mode::CARTESIAN_IMPEDANCE);
    }
  }

  if (!success) {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

void Side::reset_motion_cmd_to_current_measured() {
  if (hw_state_motion_status_.header.stamp.sec != 0) {
    // Copy current measured positions to command
    motion_cmd_.commanded_joint_position = hw_state_motion_status_.measured_joint_position;
    motion_cmd_.commanded_cartesian_pose = hw_state_motion_status_.measured_cartesian_pose;
  }
}

bool Side::validateMotionCommand() const {
  // Basic validation - check that joint positions are finite
  if (!std::isfinite(motion_cmd_.commanded_joint_position.joint_1) ||
      !std::isfinite(motion_cmd_.commanded_joint_position.joint_2) ||
      !std::isfinite(motion_cmd_.commanded_joint_position.joint_3) ||
      !std::isfinite(motion_cmd_.commanded_joint_position.joint_4) ||
      !std::isfinite(motion_cmd_.commanded_joint_position.joint_5) ||
      !std::isfinite(motion_cmd_.commanded_joint_position.joint_6) ||
      !std::isfinite(motion_cmd_.commanded_joint_position.joint_7)) {
    return false;
  }
  
  return true;
}

msg::MotionStatus Side::createMotionCommandMessage() const {
  msg::MotionStatus cmd = motion_cmd_;
  
  // Set header
  cmd.header.stamp = rclcpp::Clock().now();
  cmd.header.frame_id = side_name_ + "_arm";
  
  // Set control mode
  cmd.active_control_mode.mode = latest_control_mode_;
  
  return cmd;
}

// Helper functions to get references to joint values
double* Side::getJointPositionRef(size_t index) {
  switch (index) {
    case 0: return &hw_state_motion_status_.measured_joint_position.joint_1;
    case 1: return &hw_state_motion_status_.measured_joint_position.joint_2;
    case 2: return &hw_state_motion_status_.measured_joint_position.joint_3;
    case 3: return &hw_state_motion_status_.measured_joint_position.joint_4;
    case 4: return &hw_state_motion_status_.measured_joint_position.joint_5;
    case 5: return &hw_state_motion_status_.measured_joint_position.joint_6;
    case 6: return &hw_state_motion_status_.measured_joint_position.joint_7;
    default: throw std::runtime_error("Invalid joint index: " + std::to_string(index));
  }
}

double* Side::getJointVelocityRef(size_t index) {
  switch (index) {
    case 0: return &hw_state_motion_status_.measured_joint_velocity.joint_1;
    case 1: return &hw_state_motion_status_.measured_joint_velocity.joint_2;
    case 2: return &hw_state_motion_status_.measured_joint_velocity.joint_3;
    case 3: return &hw_state_motion_status_.measured_joint_velocity.joint_4;
    case 4: return &hw_state_motion_status_.measured_joint_velocity.joint_5;
    case 5: return &hw_state_motion_status_.measured_joint_velocity.joint_6;
    case 6: return &hw_state_motion_status_.measured_joint_velocity.joint_7;
    default: throw std::runtime_error("Invalid joint index: " + std::to_string(index));
  }
}

double* Side::getJointEffortRef(size_t index) {
  switch (index) {
    case 0: return &hw_state_motion_status_.measured_joint_torque.joint_1;
    case 1: return &hw_state_motion_status_.measured_joint_torque.joint_2;
    case 2: return &hw_state_motion_status_.measured_joint_torque.joint_3;
    case 3: return &hw_state_motion_status_.measured_joint_torque.joint_4;
    case 4: return &hw_state_motion_status_.measured_joint_torque.joint_5;
    case 5: return &hw_state_motion_status_.measured_joint_torque.joint_6;
    case 6: return &hw_state_motion_status_.measured_joint_torque.joint_7;
    default: throw std::runtime_error("Invalid joint index: " + std::to_string(index));
  }
}

double* Side::getJointPositionCmdRef(size_t index) {
  switch (index) {
    case 0: return &motion_cmd_.commanded_joint_position.joint_1;
    case 1: return &motion_cmd_.commanded_joint_position.joint_2;
    case 2: return &motion_cmd_.commanded_joint_position.joint_3;
    case 3: return &motion_cmd_.commanded_joint_position.joint_4;
    case 4: return &motion_cmd_.commanded_joint_position.joint_5;
    case 5: return &motion_cmd_.commanded_joint_position.joint_6;
    case 6: return &motion_cmd_.commanded_joint_position.joint_7;
    default: throw std::runtime_error("Invalid joint index: " + std::to_string(index));
  }
}

// Methods for hardware interface
bool Side::hasLatestMotionStatus() const {
  return has_motion_status_;
}

bool Side::hasLatestGripperStatus() const {
  return has_gripper_status_;
}

const msg::MotionStatus& Side::getLatestMotionStatus() const {
  return latest_motion_status_;
}

const msg::Robotiq3FingerStatus& Side::getLatestGripperStatus() const {
  return latest_gripper_status_;
}

// Static helper functions for accessing joint fields in messages
double Side::getJoint1Position(const victor_hardware_interfaces::msg::JointValueQuantity& jp) {
  return jp.joint_1;
}

double Side::getJoint2Position(const victor_hardware_interfaces::msg::JointValueQuantity& jp) {
  return jp.joint_2;
}

double Side::getJoint3Position(const victor_hardware_interfaces::msg::JointValueQuantity& jp) {
  return jp.joint_3;
}

double Side::getJoint4Position(const victor_hardware_interfaces::msg::JointValueQuantity& jp) {
  return jp.joint_4;
}

double Side::getJoint5Position(const victor_hardware_interfaces::msg::JointValueQuantity& jp) {
  return jp.joint_5;
}

double Side::getJoint6Position(const victor_hardware_interfaces::msg::JointValueQuantity& jp) {
  return jp.joint_6;
}

double Side::getJoint7Position(const victor_hardware_interfaces::msg::JointValueQuantity& jp) {
  return jp.joint_7;
}

}  // namespace victor_sim_hardware
