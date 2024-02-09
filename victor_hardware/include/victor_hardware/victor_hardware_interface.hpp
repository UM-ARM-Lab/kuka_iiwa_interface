// A ros2_control compliant hardware interface class for Victor
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <lcm/lcm-cpp.hpp>
#include <memory>
#include <rclcpp/executor.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <thread>
#include <vector>
#include <victor_hardware/lcm_listener.hpp>
#include <victor_hardware_interfaces/srv/get_control_mode.hpp>
#include <victor_hardware_interfaces/srv/set_control_mode.hpp>
#include <victor_lcm_interface/control_mode_parameters.hpp>
#include <victor_lcm_interface/motion_command.hpp>
#include <victor_lcm_interface/motion_status.hpp>
#include <victor_lcm_interface/robotiq_3finger_status.hpp>

// #include "data_tamer/data_tamer.hpp"
// #include "data_tamer/sinks/mcap_sink.hpp"

#define VICTOR_HARDWARE_PUBLIC __attribute__((visibility("default")))

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace victor_hardware {
class VictorHardwareInterface : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(VictorHardwareInterface);

  VICTOR_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  VICTOR_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  VICTOR_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  VICTOR_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  VICTOR_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  VICTOR_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  VICTOR_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // Communication
  std::shared_ptr<lcm::LCM> left_send_lcm_ptr_;
  std::shared_ptr<lcm::LCM> left_recv_lcm_ptr_;
  std::shared_ptr<lcm::LCM> right_send_lcm_ptr_;
  std::shared_ptr<lcm::LCM> right_recv_lcm_ptr_;

  // Listeners
  std::unique_ptr<LcmListener<victor_lcm_interface::motion_status>> left_motion_status_listener_;
  std::unique_ptr<LcmListener<victor_lcm_interface::motion_status>> right_motion_status_listener_;
  std::unique_ptr<LcmListener<victor_lcm_interface::control_mode_parameters>> left_control_mode_listener_;
  std::unique_ptr<LcmListener<victor_lcm_interface::control_mode_parameters>> right_control_mode_listener_;
  std::unique_ptr<LcmListener<victor_lcm_interface::robotiq_3finger_status>> left_gripper_status_listener_;
  std::unique_ptr<LcmListener<victor_lcm_interface::robotiq_3finger_status>> right_gripper_status_listener_;

  // thread for LCM handling
  std::thread lcm_thread_;
  std::atomic<bool> lcm_thread_running_;

  rclcpp::Service<victor_hardware_interfaces::srv::SetControlMode>::SharedPtr set_left_control_mode_srv_;
  rclcpp::Service<victor_hardware_interfaces::srv::SetControlMode>::SharedPtr set_right_control_mode_srv_;

  // thread for ROS handling
  std::thread ros_thread_;

  void LCMThread();

  void RosThread();

  std::vector<double> hw_pos_cmds_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_effort_;
  std::vector<double> hw_states_external_torque_sensor_;
  std::array<double, 6> left_hw_ft_;
  std::array<double, 6> right_hw_ft_;

  //  std::shared_ptr<DataTamer::MCAPSink> sink_;
  //  std::shared_ptr<DataTamer::LogChannel> channel_;
  //  DataTamer::RegistrationID value_;

  rclcpp::Executor::SharedPtr executor_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace victor_hardware