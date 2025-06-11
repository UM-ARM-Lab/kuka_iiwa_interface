// A ros2_control compliant hardware interface class for Victor Simulator
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <memory>
#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <thread>
#include <vector>
#include <victor_sim_hardware/side.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace victor_sim_hardware {
class VictorSimHardwareInterface : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(VictorSimHardwareInterface)

  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

 private:
  // Node for custom ROS API that goes beyond what ros2 control allows
  std::shared_ptr<rclcpp::Node> node_;

  // This class does the pub-sub to ROS topics for simulator communication
  Side left{"left"};
  Side right{"right"};

  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_effort_;
  std::vector<double> hw_states_cmd_position_;
  std::vector<double> hw_states_external_effort_;
  std::vector<double> hw_states_external_torque_sensor_;

  bool enable_left_arm_{false};  // Flag to enable/disable left arm control
  bool enable_right_arm_{false}; // Flag to enable/disable right arm control
};

}  // namespace victor_sim_hardware