#include <arc_utilities/arc_helpers.hpp>
// ROS message headers
#include <victor_hardware_interface_msgs/Robotiq3FingerCommand.h>
#include <victor_hardware_interface_msgs/Robotiq3FingerStatus.h>
// LCM type headers
#include <victor_hardware_interface/robotiq_3finger_command.hpp>
#include <victor_hardware_interface/robotiq_3finger_status.hpp>
// LCM
#include <lcm/lcm-cpp.hpp>

#ifndef ROBOTIQ_3FINGER_HARDWARE_INTERFACE_HPP
#define ROBOTIQ_3FINGER_HARDWARE_INTERFACE_HPP

namespace victor_hardware_interface
{
    namespace msgs = victor_hardware_interface_msgs;

    /////////////////////////////////////////////////////////////////////////////////
    // Robotiq3FingerHardwardInterface class declaration
    /////////////////////////////////////////////////////////////////////////////////

    class Robotiq3FingerHardwareInterface
    {
    protected:

        std::shared_ptr<lcm::LCM> send_lcm_ptr_;
        std::shared_ptr<lcm::LCM> recv_lcm_ptr_;
        std::string command_channel_name_;
        std::string status_channel_name_;
        std::function<void(const msgs::Robotiq3FingerStatus&)> status_callback_fn_;

        void internalStatusLCMCallback(
                const lcm::ReceiveBuffer* buffer,
                const std::string& channel,
                const robotiq_3finger_status* status_msg);

    public:

        Robotiq3FingerHardwareInterface(
                const std::shared_ptr<lcm::LCM>& send_lcm_ptr,
                const std::shared_ptr<lcm::LCM>& recv_lcm_ptr,
                const std::string& command_channel_name,
                const std::string& status_channel_name,
                const std::function<void(const msgs::Robotiq3FingerStatus&)>& status_callback_fn);

        bool sendCommandMessage(const msgs::Robotiq3FingerCommand& command);
    };

    /////////////////////////////////////////////////////////////////////////////////
    // Ros and LCM convert helper functions
    /////////////////////////////////////////////////////////////////////////////////

    robotiq_3finger_actuator_command fingerCommandRosToLcm(const msgs::Robotiq3FingerActuatorCommand& finger_command);

    msgs::Robotiq3FingerActuatorStatus fingerStatusLcmToRos(const robotiq_3finger_actuator_status& finger_status);

    msgs::Robotiq3FingerObjectStatus objectStatusLcmToRos(const robotiq_3finger_object_status& object_status);

    msgs::Robotiq3FingerStatus statusLcmToRos(const robotiq_3finger_status& status);

    robotiq_3finger_command commandRosToLcm(const msgs::Robotiq3FingerCommand& command);

}
#endif // ROBOTIQ_3FINGER_HARDWARE_INTERFACE_HPP
