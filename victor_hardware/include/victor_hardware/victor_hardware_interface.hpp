// A ros2_control compliant hardware interface class for Victor
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <lcm/lcm-cpp.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <thread>
#include <vector>
#include <std_srvs/srv/set_bool.hpp>
#include <victor_hardware/lcm_listener.hpp>
#include <victor_hardware/async_executor.hpp>
#include <victor_lcm_interface/control_mode_parameters.hpp>
#include <victor_lcm_interface/motion_command.hpp>
#include <victor_lcm_interface/motion_status.hpp>
#include <victor_lcm_interface/robotiq_3finger_status.hpp>

//#include "data_tamer/data_tamer.hpp"
//#include "data_tamer/sinks/mcap_sink.hpp"


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
  // Node for custom ROS API that goes outside of what ros2 control allows
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<AsyncExecutor> executor_;
  rclcpp::CallbackGroup::SharedPtr setter_callback_group_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_send_motion_command_srv_;

  // flag to disable sending MotionCommands, so that the user can do this themselves
  bool send_motion_cmd_ = true;

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

  void LCMThread();

  std::vector<double> hw_pos_cmds_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_effort_;
  std::vector<double> hw_states_external_torque_sensor_;
  std::array<double, 6> left_hw_ft_;
  std::array<double, 6> right_hw_ft_;

//  std::shared_ptr<DataTamer::MCAPSink> sink_;
//  std::shared_ptr<DataTamer::LogChannel> channel_;
//  DataTamer::RegistrationID value_;
};

}  // namespace victor_hardware