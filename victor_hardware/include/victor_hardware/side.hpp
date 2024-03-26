#pragma once

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <hardware_interface/system_interface.hpp>
#include <lcm/lcm-cpp.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <string>
#include <unordered_map>
#include <victor_hardware/kuka_control_mode_client.hpp>
#include <victor_hardware/lcm_listener.hpp>
#include <victor_hardware/validators.hpp>
#include <victor_hardware/types.hpp>
#include <victor_hardware_interfaces/msg/control_mode_parameters.hpp>
#include <victor_hardware_interfaces/msg/motion_command.hpp>
#include <victor_hardware_interfaces/msg/motion_status.hpp>
#include <victor_hardware_interfaces/msg/robotiq3_finger_command.hpp>
#include <victor_hardware_interfaces/msg/robotiq3_finger_status.hpp>
#include <victor_hardware_interfaces/srv/get_control_mode.hpp>
#include <victor_hardware_interfaces/srv/set_control_mode.hpp>
#include <victor_lcm_interface/control_mode_parameters.hpp>
#include <victor_lcm_interface/motion_command.hpp>
#include <victor_lcm_interface/motion_status.hpp>
#include <victor_lcm_interface/robotiq_3finger_status.hpp>

namespace msg = victor_hardware_interfaces::msg;
namespace srv = victor_hardware_interfaces::srv;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace victor_hardware {

class Side {
 public:
  explicit Side(std::string const& name);

  void add_command_interfaces(hardware_interface::HardwareInfo const& info,
                              std::vector<hardware_interface::CommandInterface>& command_interfaces);
  void add_state_interfaces(std::vector<hardware_interface::StateInterface>& state_interfaces);

  CallbackReturn on_init(std::shared_ptr<rclcpp::Node> const& node,
                         std::string const& send_provider, std::string const& recv_provider);

  void gripperCommandROSCallback(const msg::Robotiq3FingerCommand& command);
  void read_motion_status(const victor_lcm_interface::motion_status& status);
  void publish_motion_status(victor_lcm_interface::motion_status const& msg);
  void publish_gripper_status(victor_lcm_interface::robotiq_3finger_status const& msg);
  [[nodiscard]] hardware_interface::return_type send_motion_command();

  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces);

  std::string side_name_;

  victor_lcm_interface::motion_command motion_cmd_{};
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
  victor_lcm_interface::motion_status hw_state_motion_status_;

  LCMPtr send_lcm_ptr_;
  LCMPtr recv_lcm_ptr_;

  std::string cartesian_control_frame_;

  std::unique_ptr<LcmListener<victor_lcm_interface::motion_status>> motion_status_listener_;
  std::unique_ptr<LcmListener<victor_lcm_interface::robotiq_3finger_status>> gripper_status_listener_;
  rclcpp::Service<srv::SetControlMode>::SharedPtr set_control_mode_server_;
  rclcpp::Service<srv::GetControlMode>::SharedPtr get_control_mode_server_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;

  std::shared_ptr<KukaControlModeClientNode> control_mode_client_;

  // ROS API
  rclcpp::Publisher<msg::MotionStatus>::SharedPtr motion_status_pub_;
  rclcpp::Publisher<msg::ControlModeParameters>::SharedPtr control_mode_params_pub_;
  rclcpp::Publisher<msg::Robotiq3FingerStatus>::SharedPtr gripper_status_pub_;
  rclcpp::Subscription<msg::Robotiq3FingerCommand>::SharedPtr gripper_command_sub_;

  // callback groups for each ROS thing
  rclcpp::CallbackGroup::SharedPtr getter_callback_group_;
  rclcpp::CallbackGroup::SharedPtr setter_callback_group_;


 private:
  rclcpp::Logger logger_;
  bool has_active_controller_{false};
  int8_t latest_control_mode_ = victor_lcm_interface::control_mode::JOINT_POSITION;
  void reset_motion_cmd_to_current_measured();
};

}  // namespace victor_hardware
