// This file uses pybind11 to create a python interface to the specific LCM types we use to talk to the robot.
// In contrast to generating python messages using lcm-gen, this file allows us to use the same C++ code to
// publish and subscribe to messages in python.
// When I tried regenerating them and using the python LCM library, I could get it to work.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <lcm/lcm-cpp.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <victor_hardware/constants.hpp>
#include <victor_hardware/lcm_listener.hpp>
#include <victor_lcm_interface/control_mode_parameters.hpp>
#include <victor_lcm_interface/motion_command.hpp>
#include <victor_lcm_interface/motion_status.hpp>
#include <victor_lcm_interface/robotiq_3finger_command.hpp>
#include <victor_lcm_interface/robotiq_3finger_status.hpp>

namespace py = pybind11;

class IiwaLCMInterface {
 public:
  IiwaLCMInterface(std::string const& recv_provider, std::string const& send_provider) {
    recv_lcm_ptr_ = std::make_shared<lcm::LCM>(recv_provider);
    send_lcm_ptr_ = std::make_shared<lcm::LCM>(send_provider);

    if (!recv_lcm_ptr_->good()) {
      throw std::runtime_error("Receive LCM interface is not good");
    }
    if (!send_lcm_ptr_->good()) {
      throw std::runtime_error("Send LCM interface is not good");
    }

    control_mode_listener_ = std::make_unique<LcmListener<victor_lcm_interface::control_mode_parameters>>(
        recv_lcm_ptr_, DEFAULT_CONTROL_MODE_STATUS_CHANNEL);
    motion_status_listener_ = std::make_unique<LcmListener<victor_lcm_interface::motion_status>>(
        recv_lcm_ptr_, DEFAULT_MOTION_STATUS_CHANNEL);
    gripper_status_listener_ = std::make_unique<LcmListener<victor_lcm_interface::robotiq_3finger_status>>(
        recv_lcm_ptr_, DEFAULT_GRIPPER_STATUS_CHANNEL);

    lcm_thread_running_ = true;
    lcm_thread_ = std::thread(&IiwaLCMInterface::handle_lcm, this);
  }

  void handle_lcm() {
    while (lcm_thread_running_) {
      recv_lcm_ptr_->handleTimeout(1000);
    }
  }

  ~IiwaLCMInterface() {
    lcm_thread_running_ = false;
    lcm_thread_.join();
  }

  void send_control_mode_params(victor_lcm_interface::control_mode_parameters const& control_mode_params) {
    send_lcm_ptr_->publish(DEFAULT_CONTROL_MODE_COMMAND_CHANNEL, &control_mode_params);
  }

  void send_motion_command(victor_lcm_interface::motion_command const& command) {
    send_lcm_ptr_->publish(DEFAULT_MOTION_COMMAND_CHANNEL, &command);
  }

  void send_gripper_command(victor_lcm_interface::robotiq_3finger_command const& command) {
    send_lcm_ptr_->publish(DEFAULT_GRIPPER_COMMAND_CHANNEL, &command);
  }

 private:
  std::shared_ptr<lcm::LCM> recv_lcm_ptr_;
  std::shared_ptr<lcm::LCM> send_lcm_ptr_;

  std::thread lcm_thread_;
  std::atomic<bool> lcm_thread_running_;

  std::unique_ptr<LcmListener<victor_lcm_interface::control_mode_parameters>> control_mode_listener_;
  std::unique_ptr<LcmListener<victor_lcm_interface::motion_status>> motion_status_listener_;
  std::unique_ptr<LcmListener<victor_lcm_interface::robotiq_3finger_status>> gripper_status_listener_;
};

struct VictorLCMInterface {
  IiwaLCMInterface left{LEFT_RECV_PROVIDER, LEFT_SEND_PROVIDER};
  IiwaLCMInterface right{RIGHT_RECV_PROVIDER, RIGHT_SEND_PROVIDER};
};

PYBIND11_MODULE(lcm_python_bindings, m) {
  m.doc() = "LCM Python Bindings";

  py::class_<IiwaLCMInterface>(m, "IiwaLCMInterface")
      .def(py::init<std::string, std::string>())
      .def("send_control_mode_params", &IiwaLCMInterface::send_control_mode_params)
      .def("send_motion_command", &IiwaLCMInterface::send_motion_command)
      .def("send_gripper_command", &IiwaLCMInterface::send_gripper_command);

  py::class_<VictorLCMInterface>(m, "VictorLCMInterface")
      .def(py::init<>())
      .def_readonly("left", &VictorLCMInterface::left)
      .def_readonly("right", &VictorLCMInterface::right);

  // Bindings for the LCM structs
  py::class_<victor_lcm_interface::control_mode_parameters>(m, "ControlModeParameters")
      .def(py::init<>())
      .def_readwrite("cartesian_control_mode_limits",
                     &victor_lcm_interface::control_mode_parameters::cartesian_control_mode_limits)
      .def_readwrite("cartesian_impedance_params",
                     &victor_lcm_interface::control_mode_parameters::cartesian_impedance_params)
      .def_readwrite("cartesian_path_execution_params",
                     &victor_lcm_interface::control_mode_parameters::cartesian_path_execution_params)
      .def_readwrite("joint_impedance_params", &victor_lcm_interface::control_mode_parameters::joint_impedance_params)
      .def_readwrite("joint_path_execution_params",
                     &victor_lcm_interface::control_mode_parameters::joint_path_execution_params)
      .def_readwrite("control_mode", &victor_lcm_interface::control_mode_parameters::control_mode);

  py::class_<victor_lcm_interface::cartesian_control_mode_limits>(m, "CartesianControlModeLimits")
      .def(py::init<>())
      .def_readwrite("max_cartesian_velocity",
                     &victor_lcm_interface::cartesian_control_mode_limits::max_cartesian_velocity)
      .def_readwrite("max_cartesian_path_deviation",
                     &victor_lcm_interface::cartesian_control_mode_limits::max_path_deviation)
      .def_readwrite("max_control_force", &victor_lcm_interface::cartesian_control_mode_limits::max_control_force)
      .def_readwrite("stop_on_max_control_force",
                     &victor_lcm_interface::cartesian_control_mode_limits::stop_on_max_control_force);

  py::class_<victor_lcm_interface::cartesian_impedance_parameters>(m, "CartesianImpedanceParameters")
      .def(py::init<>())
      .def_readwrite("cartesian_stiffness", &victor_lcm_interface::cartesian_impedance_parameters::cartesian_stiffness)

      .def_readwrite("nullspace_stiffness", &victor_lcm_interface::cartesian_impedance_parameters::nullspace_stiffness)
      .def_readwrite("cartesian_damping", &victor_lcm_interface::cartesian_impedance_parameters::cartesian_damping)
      .def_readwrite("nullspace_damping", &victor_lcm_interface::cartesian_impedance_parameters::nullspace_damping);

  py::class_<victor_lcm_interface::cartesian_path_execution_parameters>(m, "CartesianPathExecutionParameters")
      .def(py::init<>())
      .def_readwrite("max_velocity", &victor_lcm_interface::cartesian_path_execution_parameters::max_velocity)
      .def_readwrite("max_acceleration", &victor_lcm_interface::cartesian_path_execution_parameters::max_acceleration)
      .def_readwrite("max_nullspace_velocity",
                     &victor_lcm_interface::cartesian_path_execution_parameters::max_nullspace_velocity)
      .def_readwrite("max_nullspace_acceleration",
                     &victor_lcm_interface::cartesian_path_execution_parameters::max_nullspace_acceleration);

  py::class_<victor_lcm_interface::cartesian_pose>(m, "CartesianPose")
      .def(py::init<>())
      .def_readwrite("xt", &victor_lcm_interface::cartesian_pose::xt)
      .def_readwrite("yt", &victor_lcm_interface::cartesian_pose::yt)
      .def_readwrite("zt", &victor_lcm_interface::cartesian_pose::zt)
      .def_readwrite("wr", &victor_lcm_interface::cartesian_pose::wr)
      .def_readwrite("xr", &victor_lcm_interface::cartesian_pose::xr)
      .def_readwrite("yr", &victor_lcm_interface::cartesian_pose::yr)
      .def_readwrite("zr", &victor_lcm_interface::cartesian_pose::zr);

  py::class_<victor_lcm_interface::cartesian_value_quantity>(m, "CartesianValueQuantity")
      .def(py::init<>())
      .def_readwrite("x", &victor_lcm_interface::cartesian_value_quantity::x)
      .def_readwrite("y", &victor_lcm_interface::cartesian_value_quantity::y)
      .def_readwrite("z", &victor_lcm_interface::cartesian_value_quantity::z)
      .def_readwrite("a", &victor_lcm_interface::cartesian_value_quantity::a)
      .def_readwrite("b", &victor_lcm_interface::cartesian_value_quantity::b)
      .def_readwrite("c", &victor_lcm_interface::cartesian_value_quantity::c);

  py::class_<victor_lcm_interface::control_mode>(m, "ControlMode")
      .def(py::init<>())
      .def_readwrite("mode", &victor_lcm_interface::control_mode::mode)
      .def_readonly_static("JOINT_POSITION", &victor_lcm_interface::control_mode::JOINT_POSITION)
      .def_readonly_static("JOINT_IMPEDANCE", &victor_lcm_interface::control_mode::JOINT_IMPEDANCE)
      .def_readonly_static("CARTESIAN_POSE", &victor_lcm_interface::control_mode::CARTESIAN_POSE)
      .def_readonly_static("CARTESIAN_IMPEDANCE", &victor_lcm_interface::control_mode::CARTESIAN_IMPEDANCE);

  py::class_<victor_lcm_interface::joint_impedance_parameters>(m, "JointImpedanceParameters")
      .def(py::init<>())
      .def_readwrite("joint_stiffness", &victor_lcm_interface::joint_impedance_parameters::joint_stiffness)
      .def_readwrite("joint_damping", &victor_lcm_interface::joint_impedance_parameters::joint_damping);

  py::class_<victor_lcm_interface::joint_path_execution_parameters>(m, "JointPathExecutionParameters")
      .def(py::init<>())
      .def_readwrite("joint_relative_velocity",
                     &victor_lcm_interface::joint_path_execution_parameters::joint_relative_velocity)
      .def_readwrite("joint_relative_acceleration",
                     &victor_lcm_interface::joint_path_execution_parameters::joint_relative_acceleration)
      .def_readwrite("override_joint_acceleration",
                     &victor_lcm_interface::joint_path_execution_parameters::override_joint_acceleration);

  py::class_<victor_lcm_interface::joint_value_quantity>(m, "JointValueQuantity")
      .def(py::init<>())
      .def_readwrite("joint_1", &victor_lcm_interface::joint_value_quantity::joint_1)
      .def_readwrite("joint_2", &victor_lcm_interface::joint_value_quantity::joint_2)
      .def_readwrite("joint_3", &victor_lcm_interface::joint_value_quantity::joint_3)
      .def_readwrite("joint_4", &victor_lcm_interface::joint_value_quantity::joint_4)
      .def_readwrite("joint_5", &victor_lcm_interface::joint_value_quantity::joint_5)
      .def_readwrite("joint_6", &victor_lcm_interface::joint_value_quantity::joint_6)

      .def_readwrite("joint_7", &victor_lcm_interface::joint_value_quantity::joint_7);

  py::class_<victor_lcm_interface::motion_command>(m, "MotionCommand")
      .def(py::init<>())
      .def_readwrite("joint_position", &victor_lcm_interface::motion_command::joint_position)
      .def_readwrite("joint_velocity", &victor_lcm_interface::motion_command::joint_velocity)
      .def_readwrite("cartesian_pose", &victor_lcm_interface::motion_command::cartesian_pose)
      .def_readwrite("control_mode", &victor_lcm_interface::motion_command::control_mode)
      .def_readwrite("timestamp", &victor_lcm_interface::motion_command::timestamp);

  py::class_<victor_lcm_interface::motion_status>(m, "MotionStatus")
      .def(py::init<>())
      .def_readwrite("measured_joint_position", &victor_lcm_interface::motion_status::measured_joint_position)
      .def_readwrite("commanded_joint_position", &victor_lcm_interface::motion_status::commanded_joint_position)
      .def_readwrite("measured_joint_velocity", &victor_lcm_interface::motion_status::measured_joint_velocity)
      .def_readwrite("measured_joint_torque", &victor_lcm_interface::motion_status::measured_joint_torque)
      .def_readwrite("estimated_external_torque", &victor_lcm_interface::motion_status::estimated_external_torque)
      .def_readwrite("estimated_external_wrench", &victor_lcm_interface::motion_status::estimated_external_wrench)
      .def_readwrite("measured_cartesian_pose_abc", &victor_lcm_interface::motion_status::measured_cartesian_pose_abc)
      .def_readwrite("commanded_cartesian_pose_abc", &victor_lcm_interface::motion_status::commanded_cartesian_pose_abc)
      .def_readwrite("measured_cartesian_pose", &victor_lcm_interface::motion_status::measured_cartesian_pose)
      .def_readwrite("commanded_cartesian_pose", &victor_lcm_interface::motion_status::commanded_cartesian_pose)
      .def_readwrite("active_control_mode", &victor_lcm_interface::motion_status::active_control_mode)
      .def_readwrite("timestamp", &victor_lcm_interface::motion_status::timestamp);

  py::class_<victor_lcm_interface::robotiq_3finger_actuator_command>(m, "Robotiq3FingerActuatorCommand")
      .def(py::init<>())
      .def_readwrite("timestamp", &victor_lcm_interface::robotiq_3finger_actuator_command::timestamp)
      .def_readwrite("position", &victor_lcm_interface::robotiq_3finger_actuator_command::position)
      .def_readwrite("speed", &victor_lcm_interface::robotiq_3finger_actuator_command::speed)
      .def_readwrite("force", &victor_lcm_interface::robotiq_3finger_actuator_command::force);

  py::class_<victor_lcm_interface::robotiq_3finger_actuator_status>(m, "Robotiq3FingerActuatorStatus")
      .def(py::init<>())
      .def_readwrite("timestamp", &victor_lcm_interface::robotiq_3finger_actuator_status::timestamp)
      .def_readwrite("position_request", &victor_lcm_interface::robotiq_3finger_actuator_status::position_request)
      .def_readwrite("position", &victor_lcm_interface::robotiq_3finger_actuator_status::position)
      .def_readwrite("current", &victor_lcm_interface::robotiq_3finger_actuator_status::current);


  py::class_<victor_lcm_interface::robotiq_3finger_command>(m, "Robotiq3FingerCommand")
      .def(py::init<>())
      .def_readwrite("timestamp", &victor_lcm_interface::robotiq_3finger_command::timestamp)
      .def_readwrite("finger_a_command", &victor_lcm_interface::robotiq_3finger_command::finger_a_command)
      .def_readwrite("finger_b_command", &victor_lcm_interface::robotiq_3finger_command::finger_b_command)
      .def_readwrite("finger_c_command", &victor_lcm_interface::robotiq_3finger_command::finger_c_command)
      .def_readwrite("scissor_command", &victor_lcm_interface::robotiq_3finger_command::scissor_command);

  py::class_<victor_lcm_interface::robotiq_3finger_object_status>(m, "Robotiq3FingerObjectStatus")
      .def(py::init<>())
      .def_readwrite("timestamp", &victor_lcm_interface::robotiq_3finger_object_status::timestamp)
      .def_readwrite("status", &victor_lcm_interface::robotiq_3finger_object_status::status)
      .def_readonly_static("IN_MOTION", &victor_lcm_interface::robotiq_3finger_object_status::IN_MOTION)
      .def_readonly_static("AT_REQUESTED", &victor_lcm_interface::robotiq_3finger_object_status::AT_REQUESTED)
      .def_readonly_static("STOPPED", &victor_lcm_interface::robotiq_3finger_object_status::STOPPED)
      .def_readonly_static("CONTACT_OPENING", &victor_lcm_interface::robotiq_3finger_object_status::CONTACT_OPENING)
      .def_readonly_static("CONTACT_CLOSING", &victor_lcm_interface::robotiq_3finger_object_status::CONTACT_CLOSING);


  py::class_<victor_lcm_interface::robotiq_3finger_status>(m, "Robotiq3FingerStatus")
      .def(py::init<>())
      .def_readwrite("timestamp", &victor_lcm_interface::robotiq_3finger_status::timestamp)
      .def_readwrite("finger_a_status", &victor_lcm_interface::robotiq_3finger_status::finger_a_status)
      .def_readwrite("finger_b_status", &victor_lcm_interface::robotiq_3finger_status::finger_b_status)
      .def_readwrite("finger_c_status", &victor_lcm_interface::robotiq_3finger_status::finger_c_status)
      .def_readwrite("scissor_status", &victor_lcm_interface::robotiq_3finger_status::scissor_status)
      .def_readwrite("finger_a_object_status", &victor_lcm_interface::robotiq_3finger_status::finger_a_object_status)
      .def_readwrite("finger_b_object_status", &victor_lcm_interface::robotiq_3finger_status::finger_b_object_status)
      .def_readwrite("finger_c_object_status", &victor_lcm_interface::robotiq_3finger_status::finger_c_object_status)
      .def_readwrite("scissor_object_status", &victor_lcm_interface::robotiq_3finger_status::scissor_object_status)
      .def_readwrite("initialization_status", &victor_lcm_interface::robotiq_3finger_status::initialization_status)
      .def_readwrite("gripper_action_status", &victor_lcm_interface::robotiq_3finger_status::gripper_action_status)
      .def_readwrite("gripper_system_status", &victor_lcm_interface::robotiq_3finger_status::gripper_system_status)
      .def_readwrite("gripper_motion_status", &victor_lcm_interface::robotiq_3finger_status::gripper_motion_status)
      .def_readwrite("gripper_fault_status", &victor_lcm_interface::robotiq_3finger_status::gripper_fault_status)
      .def_readonly_static("GRIPPER_RESET", &victor_lcm_interface::robotiq_3finger_status::GRIPPER_RESET)
      .def_readonly_static("GRIPPER_ACTIVATION", &victor_lcm_interface::robotiq_3finger_status::GRIPPER_ACTIVATION)
      .def_readonly_static("GRIPPER_STOPPED_OR_BUSY", &victor_lcm_interface::robotiq_3finger_status::GRIPPER_STOPPED_OR_BUSY)
      .def_readonly_static("GRIPPER_GOTO", &victor_lcm_interface::robotiq_3finger_status::GRIPPER_GOTO)
      .def_readonly_static("GRIPPER_RESET_OR_AUTO_RELEASE", &victor_lcm_interface::robotiq_3finger_status::GRIPPER_RESET_OR_AUTO_RELEASE)
      .def_readonly_static("GRIPPER_ACTIVATION_IN_PROGRESS", &victor_lcm_interface::robotiq_3finger_status::GRIPPER_ACTIVATION_IN_PROGRESS)
      .def_readonly_static("GRIPPER_MODE_CHANGE_IN_PROGRESS", &victor_lcm_interface::robotiq_3finger_status::GRIPPER_MODE_CHANGE_IN_PROGRESS)
      .def_readonly_static("GRIPPER_ACTIVATION_MODE_CHANGE_COMPLETE", &victor_lcm_interface::robotiq_3finger_status::GRIPPER_ACTIVATION_MODE_CHANGE_COMPLETE)
      .def_readonly_static("GRIPPER_STOPPED_UNKNOWN", &victor_lcm_interface::robotiq_3finger_status::GRIPPER_STOPPED_UNKNOWN)
      .def_readonly_static("GRIPPER_IN_MOTION", &victor_lcm_interface::robotiq_3finger_status::GRIPPER_IN_MOTION)
      .def_readonly_static("GRIPPER_ONE_OR_TWO_STOPPED_EARLY", &victor_lcm_interface::robotiq_3finger_status::GRIPPER_ONE_OR_TWO_STOPPED_EARLY)
      .def_readonly_static("GRIPPER_ALL_STOPPED_EARLY", &victor_lcm_interface::robotiq_3finger_status::GRIPPER_ALL_STOPPED_EARLY)
      .def_readonly_static("GRIPPER_ALL_AT_REQUESTED", &victor_lcm_interface::robotiq_3finger_status::GRIPPER_ALL_AT_REQUESTED)
      .def_readonly_static("NO_FAULTS", &victor_lcm_interface::robotiq_3finger_status::NO_FAULTS)
      .def_readonly_static("PRIORITY_ACTIVATION_MUST_BE_SET", &victor_lcm_interface::robotiq_3finger_status::PRIORITY_ACTIVATION_MUST_BE_SET)
      .def_readonly_static("PRIORITY_MODE_CHANGE_NEEDED", &victor_lcm_interface::robotiq_3finger_status::PRIORITY_MODE_CHANGE_NEEDED)
      .def_readonly_static("PRIORITY_NEEDS_ACTIVATION", &victor_lcm_interface::robotiq_3finger_status::PRIORITY_NEEDS_ACTIVATION)
      .def_readonly_static("MAJOR_ACTIVATION_FAULT", &victor_lcm_interface::robotiq_3finger_status::MAJOR_ACTIVATION_FAULT)
      .def_readonly_static("MAJOR_CHANGING_MODE_FAULT", &victor_lcm_interface::robotiq_3finger_status::MAJOR_CHANGING_MODE_FAULT)
      .def_readonly_static("MAJOR_AUTO_RELEASE_COMPLETE", &victor_lcm_interface::robotiq_3finger_status::MAJOR_AUTO_RELEASE_COMPLETE)
      .def_readonly_static("MINOR_COMM_CHIP_NOT_READY", &victor_lcm_interface::robotiq_3finger_status::MINOR_COMM_CHIP_NOT_READY)
      .def_readonly_static("MINOR_CHANGING_MODE_FAULT", &victor_lcm_interface::robotiq_3finger_status::MINOR_CHANGING_MODE_FAULT)
      .def_readonly_static("MINOR_AUTO_RELEASE_IN_PROGRESS", &victor_lcm_interface::robotiq_3finger_status::MINOR_AUTO_RELEASE_IN_PROGRESS);
  }
}