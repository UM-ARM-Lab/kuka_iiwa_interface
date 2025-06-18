#pragma once

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <hardware_interface/system_interface.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <string>
#include <unordered_map>
#include <victor_sim_hardware/constants.hpp>
#include <victor_sim_hardware/sim_control_mode_client.hpp>
#include <victor_hardware_interfaces/msg/control_mode_parameters.hpp>
#include <victor_hardware_interfaces/msg/motion_status.hpp>
#include <victor_hardware_interfaces/msg/joint_value_quantity.hpp>
#include <victor_hardware_interfaces/msg/robotiq3_finger_command.hpp>
#include <victor_hardware_interfaces/msg/robotiq3_finger_status.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace msg = victor_hardware_interfaces::msg;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace victor_sim_hardware {

class Side {
 public:
  explicit Side(std::string const& name);

  void add_command_interfaces(hardware_interface::HardwareInfo const& info,
                              std::vector<hardware_interface::CommandInterface>& command_interfaces);
  void add_state_interfaces(std::vector<hardware_interface::StateInterface>& state_interfaces);

  CallbackReturn on_init(std::shared_ptr<rclcpp::Node> const& node);

  void gripperCommandROSCallback(const msg::Robotiq3FingerCommand& command);
  void motionStatusSimCallback(const msg::MotionStatus& status);
  void gripperStatusSimCallback(const msg::Robotiq3FingerStatus& status);
  void controlModeParametersCallback(const msg::ControlModeParameters& params);
  void read_motion_status(const msg::MotionStatus& status);
  void publish_motion_status(const msg::MotionStatus& msg);
  void publish_gripper_status(const msg::Robotiq3FingerStatus& msg);
  [[nodiscard]] hardware_interface::return_type send_motion_command();

  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces);

  // Methods for hardware interface
  bool hasLatestMotionStatus() const;
  bool hasLatestGripperStatus() const;
  const msg::MotionStatus& getLatestMotionStatus() const;
  const msg::Robotiq3FingerStatus& getLatestGripperStatus() const;
  
  // Helper functions for accessing joint fields in messages
  static double getJoint1Position(const victor_hardware_interfaces::msg::JointValueQuantity& jp);
  static double getJoint2Position(const victor_hardware_interfaces::msg::JointValueQuantity& jp);
  static double getJoint3Position(const victor_hardware_interfaces::msg::JointValueQuantity& jp);
  static double getJoint4Position(const victor_hardware_interfaces::msg::JointValueQuantity& jp);
  static double getJoint5Position(const victor_hardware_interfaces::msg::JointValueQuantity& jp);
  static double getJoint6Position(const victor_hardware_interfaces::msg::JointValueQuantity& jp);
  static double getJoint7Position(const victor_hardware_interfaces::msg::JointValueQuantity& jp);

  std::string side_name_;

  // Motion command data - using ROS message types instead of LCM
  msg::MotionStatus motion_cmd_{};
  // The value of these variables does not matter, they allow the HW IF to determine, in the switch mode functions,
  // which control mode is being switched to.
  double hw_cmd_joint_position_control_mode_;
  double hw_cmd_joint_impedance_control_mode_;
  double hw_cmd_cartesian_pose_control_mode_;
  double hw_cmd_cartesian_impedance_control_mode_;

  // These get bound to state interfaces
  std::array<double, 6> hw_state_ft_{};
  geometry_msgs::msg::Pose hw_state_cartesian_pose_;
  geometry_msgs::msg::Pose hw_state_cmd_cartesian_pose_;
  msg::MotionStatus hw_state_motion_status_;

  // ROS API (compatible with victor_hardware)
  rclcpp::Publisher<msg::MotionStatus>::SharedPtr motion_status_pub_;
  rclcpp::Publisher<msg::ControlModeParameters>::SharedPtr control_mode_params_pub_;
  rclcpp::Publisher<msg::Robotiq3FingerStatus>::SharedPtr gripper_status_pub_;
  rclcpp::Subscription<msg::Robotiq3FingerCommand>::SharedPtr gripper_command_sub_;

  std::shared_ptr<SimControlModeClient> control_mode_client_;

  // Simulator bridge communication (new for simulator)
  rclcpp::Publisher<msg::JointValueQuantity>::SharedPtr sim_motion_command_pub_;
  rclcpp::Publisher<msg::Robotiq3FingerCommand>::SharedPtr sim_gripper_command_pub_;
  rclcpp::Subscription<msg::MotionStatus>::SharedPtr sim_motion_status_sub_;
  rclcpp::Subscription<msg::Robotiq3FingerStatus>::SharedPtr sim_gripper_status_sub_;

  // callback groups for each ROS thing
  rclcpp::CallbackGroup::SharedPtr getter_callback_group_;
  rclcpp::CallbackGroup::SharedPtr setter_callback_group_;


 private:
  rclcpp::Logger logger_;
  bool has_active_controller_{false};
  int8_t latest_control_mode_{1}; // 1=JOINT_POSITION, 2=JOINT_IMPEDANCE, 3=CARTESIAN_POSE, 4=CARTESIAN_IMPEDANCE
  
  // Latest received data from simulator
  msg::MotionStatus latest_motion_status_;
  msg::Robotiq3FingerStatus latest_gripper_status_;
  bool has_motion_status_{false};
  bool has_gripper_status_{false};
  
  void reset_motion_cmd_to_current_measured();
  bool validateMotionCommand() const;
  // msg::MotionStatus createMotionCommandMessage() const;
  
  // Helper functions for accessing joint fields
  double* getJointPositionRef(size_t index);
  double* getJointVelocityRef(size_t index);
  double* getJointEffortRef(size_t index);
  double* getJointPositionCmdRef(size_t index);
};

}  // namespace victor_sim_hardware
