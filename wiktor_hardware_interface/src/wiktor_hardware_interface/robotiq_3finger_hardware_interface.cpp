#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <arc_utilities/arc_helpers.hpp>
#include <wiktor_hardware_interface/robotiq_3finger_hardware_interface.hpp>

namespace robotiq_3finger_hardware_interface
{
    Robotiq3FingerHardwareInterface::Robotiq3FingerHardwareInterface(const std::shared_ptr<lcm::LCM>& lcm_ptr, const std::string& command_channel_name, const std::string& status_channel_name, const std::function<void(const wiktor_hardware_interface::Robotiq3FingerStatus&)>& status_callback_fn) : lcm_ptr_(lcm_ptr), command_channel_name_(command_channel_name), status_channel_name_(status_channel_name), status_callback_fn_(status_callback_fn)
    {
        if (lcm_ptr_->good() != true)
        {
            throw std::invalid_argument("LCM interface is not good");
        }
        lcm_ptr_->subscribe(status_channel_name_, &Robotiq3FingerHardwareInterface::InternalStatusLCMCallback, this);
    }

    wiktor_hardware_interface::robotiq_3finger_actuator_command Robotiq3FingerHardwareInterface::ConvertFingerCommand(const wiktor_hardware_interface::Robotiq3FingerActuatorCommand& finger_command) const
    {
        wiktor_hardware_interface::robotiq_3finger_actuator_command lcm_command;
        lcm_command.position = arc_helpers::ClampValueAndWarn(finger_command.position, 0.0, 1.0);
        lcm_command.speed = arc_helpers::ClampValueAndWarn(finger_command.speed, 0.0, 1.0);
        lcm_command.force = arc_helpers::ClampValueAndWarn(finger_command.force, 0.0, 1.0);
        lcm_command.timestamp = finger_command.header.stamp.toSec();
        return lcm_command;
    }

    wiktor_hardware_interface::Robotiq3FingerActuatorStatus Robotiq3FingerHardwareInterface::ConvertFingerStatus(const wiktor_hardware_interface::robotiq_3finger_actuator_status& finger_status) const
    {
        wiktor_hardware_interface::Robotiq3FingerActuatorStatus ros_status;
        ros_status.position = finger_status.position;
        ros_status.position_request = finger_status.position_request;
        ros_status.current = finger_status.current;
        ros_status.header.stamp = ros::Time(finger_status.timestamp);
        return ros_status;
    }

    wiktor_hardware_interface::Robotiq3FingerObjectStatus Robotiq3FingerHardwareInterface::ConvertObjectStatus(const wiktor_hardware_interface::robotiq_3finger_object_status& object_status) const
    {
        wiktor_hardware_interface::Robotiq3FingerObjectStatus ros_status;
        ros_status.status = (uint8_t)object_status.status;
        ros_status.header.stamp = ros::Time(object_status.timestamp);
        return ros_status;
    }

    wiktor_hardware_interface::Robotiq3FingerStatus Robotiq3FingerHardwareInterface::ConvertStatus(const wiktor_hardware_interface::robotiq_3finger_status& status) const
    {
        wiktor_hardware_interface::Robotiq3FingerStatus ros_status;
        ros_status.finger_a_status = ConvertFingerStatus(status.finger_a_status);
        ros_status.finger_b_status = ConvertFingerStatus(status.finger_b_status);
        ros_status.finger_c_status = ConvertFingerStatus(status.finger_c_status);
        ros_status.scissor_status = ConvertFingerStatus(status.scissor_status);
        ros_status.finger_a_object_status = ConvertObjectStatus(status.finger_a_object_status);
        ros_status.finger_b_object_status = ConvertObjectStatus(status.finger_b_object_status);
        ros_status.finger_c_object_status = ConvertObjectStatus(status.finger_c_object_status);
        ros_status.scissor_object_status = ConvertObjectStatus(status.scissor_object_status);
        ros_status.gripper_action_status = (uint8_t)status.gripper_action_status;
        ros_status.gripper_system_status = (uint8_t)status.gripper_system_status;
        ros_status.gripper_motion_status = (uint8_t)status.gripper_motion_status;
        ros_status.gripper_fault_status = (uint8_t)status.gripper_fault_status;
        ros_status.initialization_status = (uint8_t)status.initialization_status;
        ros_status.header.stamp = ros::Time(status.timestamp);
        return ros_status;
    }

    wiktor_hardware_interface::robotiq_3finger_command Robotiq3FingerHardwareInterface::ConvertCommand(const wiktor_hardware_interface::Robotiq3FingerCommand& command) const
    {
        wiktor_hardware_interface::robotiq_3finger_command lcm_command;
        lcm_command.finger_a_command = ConvertFingerCommand(command.finger_a_command);
        lcm_command.finger_b_command = ConvertFingerCommand(command.finger_b_command);
        lcm_command.finger_c_command = ConvertFingerCommand(command.finger_c_command);
        lcm_command.scissor_command = ConvertFingerCommand(command.scissor_command);
        lcm_command.timestamp = command.header.stamp.toSec();
        return lcm_command;
    }

    bool Robotiq3FingerHardwareInterface::SendCommandMessage(const wiktor_hardware_interface::Robotiq3FingerCommand& command)
    {
        const wiktor_hardware_interface::robotiq_3finger_command lcm_command = ConvertCommand(command);
        const int ret = lcm_ptr_->publish(command_channel_name_, &lcm_command);
        if (ret == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void Robotiq3FingerHardwareInterface::InternalStatusLCMCallback(const lcm::ReceiveBuffer* buffer, const std::string& channel, const wiktor_hardware_interface::robotiq_3finger_status* status_msg)
    {
        UNUSED(buffer);
        UNUSED(channel);
        const wiktor_hardware_interface::Robotiq3FingerStatus ros_status = ConvertStatus(*status_msg);
        status_callback_fn_(ros_status);
    }
}
