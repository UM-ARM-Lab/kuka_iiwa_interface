#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <arc_utilities/arc_helpers.hpp>
// ROS message headers
#include <victor_hardware_interface/ControlModeCommand.h>
#include <victor_hardware_interface/ControlModeStatus.h>
#include <victor_hardware_interface/MotionCommand.h>
#include <victor_hardware_interface/MotionStatus.h>
// LCM type headers
#include <victor_hardware_interface/control_mode_command.hpp>
#include <victor_hardware_interface/control_mode_status.hpp>
#include <victor_hardware_interface/motion_command.hpp>
#include <victor_hardware_interface/motion_status.hpp>
// LCM
#include <lcm/lcm-cpp.hpp>

#ifndef IIWA_HARDWARE_INTERFACE_HPP
#define IIWA_HARDWARE_INTERFACE_HPP

namespace iiwa_hardware_interface
{
    class IIWAHardwareInterface
    {
    protected:

        std::shared_ptr<lcm::LCM> send_lcm_ptr_;
        std::shared_ptr<lcm::LCM> recv_lcm_ptr_;
        std::string motion_command_channel_name_;
        std::string motion_status_channel_name_;
        std::function<void(const victor_hardware_interface::MotionStatus&)> motion_status_callback_fn_;
        std::string control_mode_command_channel_name_;
        std::string control_mode_status_channel_name_;
        std::function<void(const victor_hardware_interface::ControlModeStatus&)> control_mode_status_callback_fn_;

        victor_hardware_interface::JointImpedanceParameters ConvertJointImpedanceParameters(const victor_hardware_interface::joint_impedance_parameters& joint_impedance_params) const;

        victor_hardware_interface::joint_impedance_parameters ConvertJointImpedanceParameters(const victor_hardware_interface::JointImpedanceParameters& joint_impedance_params) const;

        victor_hardware_interface::CartesianImpedanceParameters ConvertCartesianImpedanceParameters(const victor_hardware_interface::cartesian_impedance_parameters& cartesian_impedance_params) const;

        victor_hardware_interface::cartesian_impedance_parameters ConvertCartesianImpedanceParameters(const victor_hardware_interface::CartesianImpedanceParameters& cartesian_impedance_params) const;

        victor_hardware_interface::JointPathExecutionParameters ConvertJointPathExecutionParameters(const victor_hardware_interface::joint_path_execution_parameters& path_execution_params) const;

        victor_hardware_interface::joint_path_execution_parameters ConvertJointPathExecutionParameters(const victor_hardware_interface::JointPathExecutionParameters& path_execution_params) const;

        victor_hardware_interface::CartesianPathExecutionParameters ConvertCartesianPathExecutionParameters(const victor_hardware_interface::cartesian_path_execution_parameters& path_execution_params) const;

        victor_hardware_interface::cartesian_path_execution_parameters ConvertCartesianPathExecutionParameters(const victor_hardware_interface::CartesianPathExecutionParameters& path_execution_params) const;

        victor_hardware_interface::motion_command ConvertMotionCommand(const victor_hardware_interface::MotionCommand& motion_command) const;

        victor_hardware_interface::MotionStatus ConvertMotionStatus(const victor_hardware_interface::motion_status& motion_status) const;

        victor_hardware_interface::control_mode_command ConvertControlModeCommand(const victor_hardware_interface::ControlModeCommand& control_mode_command) const;

        victor_hardware_interface::ControlModeStatus ConvertControlModeStatus(const victor_hardware_interface::control_mode_status& control_mode_status) const;

        void InternalMotionStatusLCMCallback(const lcm::ReceiveBuffer* buffer, const std::string& channel, const victor_hardware_interface::motion_status* status_msg);

        void InternalControlModeStatusLCMCallback(const lcm::ReceiveBuffer* buffer, const std::string& channel, const victor_hardware_interface::control_mode_status* status_msg);

    public:

        static inline victor_hardware_interface::joint_value_quantity ConvertJVQfromROStoLCM(const victor_hardware_interface::JointValueQuantity& ros_jvq)
        {
            victor_hardware_interface::joint_value_quantity lcm_jvq;
            lcm_jvq.joint_1 = ros_jvq.joint_1;
            lcm_jvq.joint_2 = ros_jvq.joint_2;
            lcm_jvq.joint_3 = ros_jvq.joint_3;
            lcm_jvq.joint_4 = ros_jvq.joint_4;
            lcm_jvq.joint_5 = ros_jvq.joint_5;
            lcm_jvq.joint_6 = ros_jvq.joint_6;
            lcm_jvq.joint_7 = ros_jvq.joint_7;
            return lcm_jvq;
        }

        static inline victor_hardware_interface::JointValueQuantity ConvertJVQfromLCMtoROS(const victor_hardware_interface::joint_value_quantity& lcm_jvq)
        {
            victor_hardware_interface::JointValueQuantity ros_jvq;
            ros_jvq.joint_1 = lcm_jvq.joint_1;
            ros_jvq.joint_2 = lcm_jvq.joint_2;
            ros_jvq.joint_3 = lcm_jvq.joint_3;
            ros_jvq.joint_4 = lcm_jvq.joint_4;
            ros_jvq.joint_5 = lcm_jvq.joint_5;
            ros_jvq.joint_6 = lcm_jvq.joint_6;
            ros_jvq.joint_7 = lcm_jvq.joint_7;
            return ros_jvq;
        }

        static inline victor_hardware_interface::cartesian_value_quantity ConvertCVQfromROStoLCM(const victor_hardware_interface::CartesianValueQuantity& ros_cvq)
        {
            victor_hardware_interface::cartesian_value_quantity lcm_cvq;
            lcm_cvq.x = ros_cvq.x;
            lcm_cvq.y = ros_cvq.y;
            lcm_cvq.z = ros_cvq.z;
            lcm_cvq.a = ros_cvq.a;
            lcm_cvq.b = ros_cvq.b;
            lcm_cvq.c = ros_cvq.c;
            return lcm_cvq;
        }

        static inline victor_hardware_interface::CartesianValueQuantity ConvertCVQfromLCMtoROS(const victor_hardware_interface::cartesian_value_quantity& lcm_cvq)
        {
            victor_hardware_interface::CartesianValueQuantity ros_cvq;
            ros_cvq.x = lcm_cvq.x;
            ros_cvq.y = lcm_cvq.y;
            ros_cvq.z = lcm_cvq.z;
            ros_cvq.a = lcm_cvq.a;
            ros_cvq.b = lcm_cvq.b;
            ros_cvq.c = lcm_cvq.c;
            return ros_cvq;
        }

        IIWAHardwareInterface(const std::shared_ptr<lcm::LCM>& send_lcm_ptr, const std::shared_ptr<lcm::LCM>& recv_lcm_ptr, const std::string& motion_command_channel_name, const std::string& motion_status_channel_name, const std::function<void(const victor_hardware_interface::MotionStatus&)>& motion_status_callback_fn, const std::string& control_mode_command_channel_name, const std::string& control_mode_status_channel_name, const std::function<void(const victor_hardware_interface::ControlModeStatus&)>& control_mode_status_callback_fn);

        bool SendMotionCommandMessage(const victor_hardware_interface::MotionCommand& command);

        bool SendControlModeCommandMessage(const victor_hardware_interface::ControlModeCommand& command);
    };
}

#endif // IIWA_HARDWARE_INTERFACE_HPP
