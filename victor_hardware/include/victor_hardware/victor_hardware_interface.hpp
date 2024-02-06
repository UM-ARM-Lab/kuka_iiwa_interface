// A ros2_control compliant hardware interface class for Victor
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <lcm/lcm-cpp.hpp>
#include <memory>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <thread>
#include <vector>
#include <victor_lcm_interface/control_mode_parameters.hpp>
#include <victor_lcm_interface/motion_command.hpp>
#include <victor_lcm_interface/motion_status.hpp>

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
  // LCM callbacks
  void LeftMotionStatusCallback(const lcm::ReceiveBuffer* /*buffer*/, const std::string& /*channel*/,
                                const victor_lcm_interface::motion_status *motion_status);
  void LeftControlModeStatusCallback(const lcm::ReceiveBuffer* /*buffer*/, const std::string& /*channel*/,
                                     const victor_lcm_interface::control_mode_parameters *control_mode_parameters);

  void RightMotionStatusCallback(const lcm::ReceiveBuffer* /*buffer*/, const std::string& /*channel*/,
                                 const victor_lcm_interface::motion_status *motion_status);
  void RightControlModeStatusCallback(const lcm::ReceiveBuffer* /*buffer*/, const std::string& /*channel*/,
                                      const victor_lcm_interface::control_mode_parameters *control_mode_parameters);

  // Communication
  std::shared_ptr<lcm::LCM> left_send_lcm_ptr_;
  std::shared_ptr<lcm::LCM> left_recv_lcm_ptr_;
  std::shared_ptr<lcm::LCM> right_send_lcm_ptr_;
  std::shared_ptr<lcm::LCM> right_recv_lcm_ptr_;

  // Store the incoming LCM messages
  victor_lcm_interface::motion_status latest_left_motion_status_;
  victor_lcm_interface::control_mode latest_left_control_mode_;
  victor_lcm_interface::motion_status latest_right_motion_status_;
  victor_lcm_interface::control_mode latest_right_control_mode_;

  // thread for LCM handling
  std::thread lcm_thread_;
  std::atomic<bool> lcm_thread_running_;

  void LCMThread();

  std::mutex mutex_;

  std::vector<double> hw_pos_cmds_;
  std::vector<double> hw_vel_cmds_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_effort_;
  std::vector<double> hw_states_external_torque_sensor_;
};

}  // namespace victor_hardware