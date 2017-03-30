#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <arc_utilities/arc_helpers.hpp>
// ROS message headers
#include <wiktor_hardware_interface/Robotiq3FingerCommand.h>
#include <wiktor_hardware_interface/Robotiq3FingerStatus.h>
// LCM type headers
#include <wiktor_hardware_interface/robotiq_3finger_command.hpp>
#include <wiktor_hardware_interface/robotiq_3finger_status.hpp>
// LCM
#include <lcm/lcm-cpp.hpp>

#ifndef ROBOTIQ_3FINGER_HARDWARE_INTERFACE_HPP
#define ROBOTIQ_3FINGER_HARDWARE_INTERFACE_HPP

namespace robotiq_3finger_hardware_interface
{
    class Robotiq3FingerHardwareInterface
    {
    protected:

        std::shared_ptr<lcm::LCM> lcm_ptr_;
        std::string command_channel_name_;
        std::string status_channel_name_;
        std::function<void(const wiktor_hardware_interface::Robotiq3FingerStatus&)> status_callback_fn_;

        wiktor_hardware_interface::robotiq_3finger_actuator_command ConvertFingerCommand(const wiktor_hardware_interface::Robotiq3FingerActuatorCommand& finger_command) const;

        wiktor_hardware_interface::Robotiq3FingerActuatorStatus ConvertFingerStatus(const wiktor_hardware_interface::robotiq_3finger_actuator_status& finger_status) const;

        wiktor_hardware_interface::Robotiq3FingerObjectStatus ConvertObjectStatus(const wiktor_hardware_interface::robotiq_3finger_object_status& object_status) const;

        wiktor_hardware_interface::Robotiq3FingerStatus ConvertStatus(const wiktor_hardware_interface::robotiq_3finger_status& status) const;

        wiktor_hardware_interface::robotiq_3finger_command ConvertCommand(const wiktor_hardware_interface::Robotiq3FingerCommand& command) const;

        void InternalStatusLCMCallback(const lcm::ReceiveBuffer* buffer, const std::string& channel, const wiktor_hardware_interface::robotiq_3finger_status* status_msg);

    public:

        Robotiq3FingerHardwareInterface(const std::shared_ptr<lcm::LCM>& lcm_ptr, const std::string& command_channel_name, const std::string& status_channel_name, const std::function<void(const wiktor_hardware_interface::Robotiq3FingerStatus&)>& status_callback_fn);

        bool SendCommandMessage(const wiktor_hardware_interface::Robotiq3FingerCommand& command);
    };
}
#endif // ROBOTIQ_3FINGER_HARDWARE_INTERFACE_HPP
