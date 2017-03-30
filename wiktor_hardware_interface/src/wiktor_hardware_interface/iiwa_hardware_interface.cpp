#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <arc_utilities/arc_helpers.hpp>
#include <wiktor_hardware_interface/iiwa_hardware_interface.hpp>

namespace iiwa_hardware_interface
{
    IIWAHardwareInterface::IIWAHardwareInterface(const std::shared_ptr<lcm::LCM>& lcm_ptr, const std::string& motion_command_channel_name, const std::string& motion_status_channel_name, const std::function<void(const wiktor_hardware_interface::MotionStatus&)>& motion_status_callback_fn, const std::string& control_mode_command_channel_name, const std::string& control_mode_status_channel_name, const std::function<void(const wiktor_hardware_interface::ControlModeStatus&)>& control_mode_status_callback_fn) : lcm_ptr_(lcm_ptr), motion_command_channel_name_(motion_command_channel_name), motion_status_channel_name_(motion_status_channel_name), motion_status_callback_fn_(motion_status_callback_fn), control_mode_command_channel_name_(control_mode_command_channel_name), control_mode_status_channel_name_(control_mode_status_channel_name), control_mode_status_callback_fn_(control_mode_status_callback_fn)
    {
        if (lcm_ptr_->good() != true)
        {
            throw std::invalid_argument("LCM interface is not good");
        }
        lcm_ptr_->subscribe(motion_status_channel_name_, &IIWAHardwareInterface::InternalMotionStatusLCMCallback, this);
        lcm_ptr_->subscribe(control_mode_status_channel_name_, &IIWAHardwareInterface::InternalControlModeStatusLCMCallback, this);
    }

    wiktor_hardware_interface::JointImpedanceParameters IIWAHardwareInterface::ConvertJointImpedanceParameters(const wiktor_hardware_interface::joint_impedance_parameters& joint_impedance_params) const
    {
        wiktor_hardware_interface::JointImpedanceParameters ros_jip;
        ros_jip.joint_damping = ConvertJVQfromLCMtoROS(joint_impedance_params.joint_damping);
        ros_jip.joint_stiffness = ConvertJVQfromLCMtoROS(joint_impedance_params.joint_stiffness);
        return ros_jip;
    }

    wiktor_hardware_interface::joint_impedance_parameters IIWAHardwareInterface::ConvertJointImpedanceParameters(const wiktor_hardware_interface::JointImpedanceParameters& joint_impedance_params) const
    {
        wiktor_hardware_interface::joint_impedance_parameters lcm_jip;
        lcm_jip.joint_damping = ConvertJVQfromROStoLCM(joint_impedance_params.joint_damping);
        lcm_jip.joint_stiffness = ConvertJVQfromROStoLCM(joint_impedance_params.joint_stiffness);
        return lcm_jip;
    }

    wiktor_hardware_interface::CartesianImpedanceParameters IIWAHardwareInterface::ConvertCartesianImpedanceParameters(const wiktor_hardware_interface::cartesian_impedance_parameters& cartesian_impedance_params) const
    {
        wiktor_hardware_interface::CartesianImpedanceParameters ros_cip;
        ros_cip.cartesian_damping = ConvertCVQfromLCMtoROS(cartesian_impedance_params.cartesian_damping);
        ros_cip.nullspace_damping = cartesian_impedance_params.nullspace_damping;
        ros_cip.cartesian_stiffness = ConvertCVQfromLCMtoROS(cartesian_impedance_params.cartesian_stiffness);
        ros_cip.nullspace_stiffness = cartesian_impedance_params.nullspace_stiffness;
        return ros_cip;
    }

    wiktor_hardware_interface::cartesian_impedance_parameters IIWAHardwareInterface::ConvertCartesianImpedanceParameters(const wiktor_hardware_interface::CartesianImpedanceParameters& cartesian_impedance_params) const
    {
        wiktor_hardware_interface::cartesian_impedance_parameters lcm_cip;
        lcm_cip.cartesian_damping = ConvertCVQfromROStoLCM(cartesian_impedance_params.cartesian_damping);
        lcm_cip.nullspace_damping = cartesian_impedance_params.nullspace_damping;
        lcm_cip.cartesian_stiffness = ConvertCVQfromROStoLCM(cartesian_impedance_params.cartesian_stiffness);
        lcm_cip.nullspace_stiffness = cartesian_impedance_params.nullspace_stiffness;
        return lcm_cip;
    }

    wiktor_hardware_interface::PathExecutionParameters IIWAHardwareInterface::ConvertPathExecutionParameters(const wiktor_hardware_interface::path_execution_parameters& path_execution_params) const
    {
        wiktor_hardware_interface::PathExecutionParameters ros_pexp;
        ros_pexp.joint_relative_acceleration = path_execution_params.joint_relative_acceleration;
        ros_pexp.joint_relative_velocity = path_execution_params.joint_relative_velocity;
        ros_pexp.override_joint_acceleration = path_execution_params.override_joint_acceleration;
        return ros_pexp;
    }

    wiktor_hardware_interface::path_execution_parameters IIWAHardwareInterface::ConvertPathExecutionParameters(const wiktor_hardware_interface::PathExecutionParameters& path_execution_params) const
    {
        wiktor_hardware_interface::path_execution_parameters lcm_pexp;
        lcm_pexp.joint_relative_acceleration = path_execution_params.joint_relative_acceleration;
        lcm_pexp.joint_relative_velocity = path_execution_params.joint_relative_velocity;
        lcm_pexp.override_joint_acceleration = path_execution_params.override_joint_acceleration;
        return lcm_pexp;
    }

    wiktor_hardware_interface::motion_command IIWAHardwareInterface::ConvertMotionCommand(const wiktor_hardware_interface::MotionCommand& motion_command) const
    {
        wiktor_hardware_interface::motion_command lcm_command;
        lcm_command.cartesian_pose = ConvertCVQfromROStoLCM(motion_command.cartesian_pose);
        lcm_command.joint_position = ConvertJVQfromROStoLCM(motion_command.joint_position);
        lcm_command.joint_velocity = ConvertJVQfromROStoLCM(motion_command.joint_velocity);
        lcm_command.command_type = (int8_t)motion_command.command_type;
        lcm_command.timestamp = motion_command.header.stamp.toSec();
        return lcm_command;
    }

    wiktor_hardware_interface::MotionStatus IIWAHardwareInterface::ConvertMotionStatus(const wiktor_hardware_interface::motion_status& motion_status) const
    {
        wiktor_hardware_interface::MotionStatus ros_status;
        ros_status.commanded_cartesian_pose = ConvertCVQfromLCMtoROS(motion_status.commanded_cartesian_pose);
        ros_status.commanded_joint_position = ConvertJVQfromLCMtoROS(motion_status.commanded_joint_position);
        ros_status.estimated_external_torque = ConvertJVQfromLCMtoROS(motion_status.estimated_external_torque);
        ros_status.estimated_external_wrench = ConvertCVQfromLCMtoROS(motion_status.estimated_external_wrench);
        ros_status.measured_cartesian_pose = ConvertCVQfromLCMtoROS(motion_status.measured_cartesian_pose);
        ros_status.measured_joint_position = ConvertJVQfromLCMtoROS(motion_status.measured_joint_position);
        ros_status.measured_joint_torque = ConvertJVQfromLCMtoROS(motion_status.measured_joint_torque);
        ros_status.measured_joint_velocity = ConvertJVQfromLCMtoROS(motion_status.measured_joint_velocity);
        ros_status.active_command_type = (uint8_t)motion_status.active_command_type;
        ros_status.header.stamp = ros::Time(motion_status.timestamp);
        return ros_status;
    }

    wiktor_hardware_interface::control_mode_command IIWAHardwareInterface::ConvertControlModeCommand(const wiktor_hardware_interface::ControlModeCommand& control_mode_command) const
    {
        wiktor_hardware_interface::control_mode_command lcm_command;
        lcm_command.cartesian_impedance_params = ConvertCartesianImpedanceParameters(control_mode_command.cartesian_impedance_params);
        lcm_command.joint_impedance_params = ConvertJointImpedanceParameters(control_mode_command.joint_impedance_params);
        lcm_command.path_execution_params = ConvertPathExecutionParameters(control_mode_command.path_execution_params);
        lcm_command.control_mode = (int8_t)control_mode_command.control_mode;
        lcm_command.timestamp = control_mode_command.header.stamp.toSec();
        return lcm_command;
    }

    wiktor_hardware_interface::ControlModeStatus IIWAHardwareInterface::ConvertControlModeStatus(const wiktor_hardware_interface::control_mode_status& control_mode_status) const
    {
        wiktor_hardware_interface::ControlModeStatus ros_status;
        ros_status.cartesian_impedance_params = ConvertCartesianImpedanceParameters(control_mode_status.cartesian_impedance_params);
        ros_status.joint_impedance_params = ConvertJointImpedanceParameters(control_mode_status.joint_impedance_params);
        ros_status.path_execution_params = ConvertPathExecutionParameters(control_mode_status.path_execution_params);
        ros_status.active_control_mode = (uint8_t)control_mode_status.active_control_mode;
        ros_status.header.stamp = ros::Time(control_mode_status.timestamp);
        return ros_status;
    }

    bool IIWAHardwareInterface::SendMotionCommandMessage(const wiktor_hardware_interface::MotionCommand& command)
    {
        const wiktor_hardware_interface::motion_command lcm_command = ConvertMotionCommand(command);
        const int ret = lcm_ptr_->publish(motion_command_channel_name_, &lcm_command);
        if (ret == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool IIWAHardwareInterface::SendControlModeCommandMessage(const wiktor_hardware_interface::ControlModeCommand& command)
    {
        const wiktor_hardware_interface::control_mode_command lcm_command = ConvertControlModeCommand(command);
        const int ret = lcm_ptr_->publish(control_mode_command_channel_name_, &lcm_command);
        if (ret == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void IIWAHardwareInterface::InternalMotionStatusLCMCallback(const lcm::ReceiveBuffer* buffer, const std::string& channel, const wiktor_hardware_interface::motion_status* status_msg)
    {
        UNUSED(buffer);
        UNUSED(channel);
        const wiktor_hardware_interface::MotionStatus ros_status = ConvertMotionStatus(*status_msg);
        motion_status_callback_fn_(ros_status);
    }

    void IIWAHardwareInterface::InternalControlModeStatusLCMCallback(const lcm::ReceiveBuffer* buffer, const std::string& channel, const wiktor_hardware_interface::control_mode_status* status_msg)
    {
        UNUSED(buffer);
        UNUSED(channel);
        const wiktor_hardware_interface::ControlModeStatus ros_status = ConvertControlModeStatus(*status_msg);
        control_mode_status_callback_fn_(ros_status);
    }
}
