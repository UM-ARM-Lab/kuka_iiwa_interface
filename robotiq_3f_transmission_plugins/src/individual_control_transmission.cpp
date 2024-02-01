// Copyright (c) 2023 Peter Mitrano
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <robotiq_3f_transmission_plugins/individual_control_transmission.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

const auto kLogger = rclcpp::get_logger("Robotiq3fGripperHardwareInterface");

double double_to_uint8(double const value)
{
  return value * 255.0;
};

namespace robotiq_3f_transmission_plugins
{

std::array<double, 3> get_finger_thetas(double const g)
{
  // Copied from URDF, very sad
  constexpr auto const theta_1_upper = 1.2218;
  constexpr auto const theta_2_upper = 1.5708;
  constexpr auto const theta_3_lower = -1.2217;

  constexpr auto const m_1 = theta_1_upper / 140.0;
  constexpr auto const m_2 = theta_2_upper / 100.0;

  if (g <= 110)
  {
    return std::array<double, 3>{ m_1 * g, 0.0, -m_1 * g };
  }
  else if (g <= 140)
  {
    return std::array<double, 3>{ m_1 * g, 0.0, theta_3_lower };
  }
  else if (g <= 240)
  {
    return std::array<double, 3>{ theta_1_upper, m_2 * (g - 140), theta_3_lower };
  }
  else
  {
    return std::array<double, 3>{ theta_1_upper, theta_2_upper, theta_3_lower };
  }
}

double get_palm_finger_pos(double const scissor_pos)
{
  constexpr auto const palm_finger_lower = -0.192;
  constexpr auto const palm_finger_upper = 0.1784;
  auto const palm_finger = scissor_pos / 255.0 * (palm_finger_upper - palm_finger_lower) + palm_finger_lower;
  return palm_finger;
}

void IndividualControlTransmission::configure(const std::vector<transmission_interface::JointHandle>& joint_handles,
                                              const std::vector<transmission_interface::ActuatorHandle>& actuator_handles)
{
  if (joint_handles.size() != num_joints())
  {
    std::string const msg =
        "Got " + std::to_string(joint_handles.size()) + " joint handles, expected " + std::to_string(num_joints());
    throw std::runtime_error(msg);
  }

  if (actuator_handles.size() != num_actuators())
  {
    std::string const msg = "Got " + std::to_string(actuator_handles.size()) + " actuator handles, expected " +
                            std::to_string(num_actuators());
    throw std::runtime_error(msg);
  }

  // FIXME: this is hard coded based on XML order -- can I just delete the XML and construct the Handles here?
  RCLCPP_DEBUG_STREAM(kLogger, "Actuators: " << actuator_handles.size() << " Joints: " << joint_handles.size());
  for (auto const& actuator : actuator_handles)
  {
    RCLCPP_DEBUG_STREAM(kLogger, "Actuator: " << actuator.get_name());
  }
  for (auto const& joint : joint_handles)
  {
    RCLCPP_DEBUG_STREAM(kLogger, "Joint: " << joint.get_name());
  }

  finger_a_actuator_ = std::make_unique<transmission_interface::ActuatorHandle>(actuator_handles[0]);
  finger_b_actuator_ = std::make_unique<transmission_interface::ActuatorHandle>(actuator_handles[1]);
  finger_c_actuator_ = std::make_unique<transmission_interface::ActuatorHandle>(actuator_handles[2]);
  scissor_actuator_ = std::make_unique<transmission_interface::ActuatorHandle>(actuator_handles[3]);

  finger_a_joint_1_ = std::make_unique<transmission_interface::JointHandle>(joint_handles[0]);
  finger_a_joint_2_ = std::make_unique<transmission_interface::JointHandle>(joint_handles[1]);
  finger_a_joint_3_ = std::make_unique<transmission_interface::JointHandle>(joint_handles[2]);
  finger_b_joint_1_ = std::make_unique<transmission_interface::JointHandle>(joint_handles[3]);
  finger_b_joint_2_ = std::make_unique<transmission_interface::JointHandle>(joint_handles[4]);
  finger_b_joint_3_ = std::make_unique<transmission_interface::JointHandle>(joint_handles[5]);
  finger_c_joint_1_ = std::make_unique<transmission_interface::JointHandle>(joint_handles[6]);
  finger_c_joint_2_ = std::make_unique<transmission_interface::JointHandle>(joint_handles[7]);
  finger_c_joint_3_ = std::make_unique<transmission_interface::JointHandle>(joint_handles[8]);
  palm_finger_c_joint_ = std::make_unique<transmission_interface::JointHandle>(joint_handles[9]);
  palm_finger_b_joint_ = std::make_unique<transmission_interface::JointHandle>(joint_handles[10]);
};

void IndividualControlTransmission::actuator_to_joint()
{
  /**
   * Equations based on "Technical Report: Use of Hybrid Systems to model the RobotiQ Adaptive Gripper"
   *
   * g is the angle of the finger actuator, and ranges from 0 to 255.
   * Theta 1, 2, and 3 are the angles of the joints, and the limits are defined in the URDF.
   * The scissor is pretty simple, and not described in the above report, so I just approximated it as linear.
   *
   * From 4.1:
   * | Phase | Motor range | Theta 1 | Theta 2 | Theta 3 |
   * |-------|-------------|---------|---------|---------|
   * | 1     | 0 <= g <= 110 | m_1*g | 0 | - m_1*g|
   * | 2     | 110 < g <= 140 | m_1*g | 0 | theta_3_min |
   * | 3     | 140 < g <= 240 | theta_1_max | m2*(g - 140) | theta_3_min |
   * | 4     | 240 < g <= 255 | theta_1_max | theta_2_max | theta_3_min |
   *
   * Where:
   *   m_1 = theta_1_max / 140
   *   m_2 = theta_2_max / 100
   */

  // the actuators get their values from the Status function of the driver, which has to use doubles (0-1)
  // so we convert them to 0-255 here, which is what the kinematics equations use.
  auto const finger_a_pos = double_to_uint8(finger_a_actuator_->get_value());
  auto const finger_b_pos = double_to_uint8(finger_b_actuator_->get_value());
  auto const finger_c_pos = double_to_uint8(finger_c_actuator_->get_value());
  auto const scissor_pos = double_to_uint8(scissor_actuator_->get_value());

  auto const finger_a_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(finger_a_pos);
  auto const finger_b_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(finger_b_pos);
  auto const finger_c_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(finger_c_pos);
  double palm_finger = robotiq_3f_transmission_plugins::get_palm_finger_pos(scissor_pos);

  finger_a_joint_1_->set_value(finger_a_thetas[0]);
  finger_a_joint_2_->set_value(finger_a_thetas[1]);
  finger_a_joint_3_->set_value(finger_a_thetas[2]);
  finger_b_joint_1_->set_value(finger_b_thetas[0]);
  finger_b_joint_2_->set_value(finger_b_thetas[1]);
  finger_b_joint_3_->set_value(finger_b_thetas[2]);
  finger_c_joint_1_->set_value(finger_c_thetas[0]);
  finger_c_joint_2_->set_value(finger_c_thetas[1]);
  finger_c_joint_3_->set_value(finger_c_thetas[2]);
  palm_finger_c_joint_->set_value(-palm_finger);
  palm_finger_b_joint_->set_value(palm_finger);
}

void IndividualControlTransmission::joint_to_actuator()
{
  // Because the robot is under-actuated, this is ill-defined!
  RCLCPP_ERROR(kLogger, "Joint to actuator conversion is ill-defined for the Robotiq 3F gripper");
};

std::size_t IndividualControlTransmission::num_actuators() const
{
  return 4;
};

std::size_t IndividualControlTransmission::num_joints() const
{
  return 11;
}

}  // namespace robotiq_3f_transmission_plugins
