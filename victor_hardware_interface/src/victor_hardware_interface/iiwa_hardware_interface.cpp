#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <arc_utilities/arc_helpers.hpp>
#include <victor_hardware_interface/iiwa_hardware_interface.hpp>

namespace iiwa_hardware_interface
{
    IIWAHardwareInterface::IIWAHardwareInterface(const std::shared_ptr<lcm::LCM>& send_lcm_ptr, const std::shared_ptr<lcm::LCM>& recv_lcm_ptr, const std::string& motion_command_channel_name, const std::string& motion_status_channel_name, const std::function<void(const victor_hardware_interface::MotionStatus&)>& motion_status_callback_fn, const std::string& control_mode_command_channel_name, const std::string& control_mode_status_channel_name, const std::function<void(const victor_hardware_interface::ControlModeStatus&)>& control_mode_status_callback_fn) : send_lcm_ptr_(send_lcm_ptr), recv_lcm_ptr_(recv_lcm_ptr), motion_command_channel_name_(motion_command_channel_name), motion_status_channel_name_(motion_status_channel_name), motion_status_callback_fn_(motion_status_callback_fn), control_mode_command_channel_name_(control_mode_command_channel_name), control_mode_status_channel_name_(control_mode_status_channel_name), control_mode_status_callback_fn_(control_mode_status_callback_fn)
    {
        if (send_lcm_ptr_->good() != true)
        {
            throw std::invalid_argument("Send LCM interface is not good");
        }
        if (recv_lcm_ptr_->good() != true)
        {
            throw std::invalid_argument("Receive LCM interface is not good");
        }
        recv_lcm_ptr_->subscribe(motion_status_channel_name_, &IIWAHardwareInterface::InternalMotionStatusLCMCallback, this);
        recv_lcm_ptr_->subscribe(control_mode_status_channel_name_, &IIWAHardwareInterface::InternalControlModeStatusLCMCallback, this);
    }

    victor_hardware_interface::JointImpedanceParameters IIWAHardwareInterface::ConvertJointImpedanceParameters(const victor_hardware_interface::joint_impedance_parameters& joint_impedance_params) const
    {
        victor_hardware_interface::JointImpedanceParameters ros_jip;
        ros_jip.joint_damping = ConvertJVQfromLCMtoROS(joint_impedance_params.joint_damping);
        ros_jip.joint_stiffness = ConvertJVQfromLCMtoROS(joint_impedance_params.joint_stiffness);
        return ros_jip;
    }

    victor_hardware_interface::joint_impedance_parameters IIWAHardwareInterface::ConvertJointImpedanceParameters(const victor_hardware_interface::JointImpedanceParameters& joint_impedance_params) const
    {
        victor_hardware_interface::joint_impedance_parameters lcm_jip;
        lcm_jip.joint_damping = ConvertJVQfromROStoLCM(joint_impedance_params.joint_damping);
        lcm_jip.joint_stiffness = ConvertJVQfromROStoLCM(joint_impedance_params.joint_stiffness);
        return lcm_jip;
    }

    victor_hardware_interface::CartesianImpedanceParameters IIWAHardwareInterface::ConvertCartesianImpedanceParameters(const victor_hardware_interface::cartesian_impedance_parameters& cartesian_impedance_params) const
    {
        victor_hardware_interface::CartesianImpedanceParameters ros_cip;
        ros_cip.cartesian_damping = ConvertCVQfromLCMtoROS(cartesian_impedance_params.cartesian_damping);
        ros_cip.nullspace_damping = cartesian_impedance_params.nullspace_damping;
        ros_cip.cartesian_stiffness = ConvertCVQfromLCMtoROS(cartesian_impedance_params.cartesian_stiffness);
        ros_cip.nullspace_stiffness = cartesian_impedance_params.nullspace_stiffness;
        return ros_cip;
    }

    victor_hardware_interface::cartesian_impedance_parameters IIWAHardwareInterface::ConvertCartesianImpedanceParameters(const victor_hardware_interface::CartesianImpedanceParameters& cartesian_impedance_params) const
    {
        victor_hardware_interface::cartesian_impedance_parameters lcm_cip;
        lcm_cip.cartesian_damping = ConvertCVQfromROStoLCM(cartesian_impedance_params.cartesian_damping);
        lcm_cip.nullspace_damping = cartesian_impedance_params.nullspace_damping;
        lcm_cip.cartesian_stiffness = ConvertCVQfromROStoLCM(cartesian_impedance_params.cartesian_stiffness);
        lcm_cip.nullspace_stiffness = cartesian_impedance_params.nullspace_stiffness;
        return lcm_cip;
    }

    victor_hardware_interface::JointPathExecutionParameters IIWAHardwareInterface::ConvertJointPathExecutionParameters(const victor_hardware_interface::joint_path_execution_parameters& path_execution_params) const
    {
        victor_hardware_interface::JointPathExecutionParameters ros_pexp;
        ros_pexp.joint_relative_acceleration = path_execution_params.joint_relative_acceleration;
        ros_pexp.joint_relative_velocity = path_execution_params.joint_relative_velocity;
        ros_pexp.override_joint_acceleration = path_execution_params.override_joint_acceleration;
        return ros_pexp;
    }

    victor_hardware_interface::joint_path_execution_parameters IIWAHardwareInterface::ConvertJointPathExecutionParameters(const victor_hardware_interface::JointPathExecutionParameters& path_execution_params) const
    {
        victor_hardware_interface::joint_path_execution_parameters lcm_pexp;
        lcm_pexp.joint_relative_acceleration = path_execution_params.joint_relative_acceleration;
        lcm_pexp.joint_relative_velocity = path_execution_params.joint_relative_velocity;
        lcm_pexp.override_joint_acceleration = path_execution_params.override_joint_acceleration;
        return lcm_pexp;
    }

    victor_hardware_interface::CartesianPathExecutionParameters IIWAHardwareInterface::ConvertCartesianPathExecutionParameters(const victor_hardware_interface::cartesian_path_execution_parameters& path_execution_params) const
    {
        victor_hardware_interface::CartesianPathExecutionParameters ros_pexp;
        ros_pexp.max_velocity = ConvertCVQfromLCMtoROS(path_execution_params.max_velocity);
        ros_pexp.max_acceleration = ConvertCVQfromLCMtoROS(path_execution_params.max_acceleration);
        ros_pexp.max_nullspace_velocity = path_execution_params.max_nullspace_velocity;
        ros_pexp.max_nullspace_acceleration = path_execution_params.max_nullspace_acceleration;
        return ros_pexp;
    }

    victor_hardware_interface::cartesian_path_execution_parameters IIWAHardwareInterface::ConvertCartesianPathExecutionParameters(const victor_hardware_interface::CartesianPathExecutionParameters& path_execution_params) const
    {
        victor_hardware_interface::cartesian_path_execution_parameters lcm_pexp;
        lcm_pexp.max_velocity = ConvertCVQfromROStoLCM(path_execution_params.max_velocity);
        lcm_pexp.max_acceleration = ConvertCVQfromROStoLCM(path_execution_params.max_acceleration);
        lcm_pexp.max_nullspace_velocity = path_execution_params.max_nullspace_velocity;
        lcm_pexp.max_nullspace_acceleration = path_execution_params.max_nullspace_acceleration;
        return lcm_pexp;
    }

    victor_hardware_interface::CartesianControlModeLimits IIWAHardwareInterface::ConvertCartesianControlModeLimits(const victor_hardware_interface::cartesian_control_mode_limits& cartesian_control_mode_limits) const
    {
        victor_hardware_interface::CartesianControlModeLimits ros_ccml;
        ros_ccml.max_cartesian_velocity = ConvertCVQfromLCMtoROS(cartesian_control_mode_limits.max_cartesian_velocity);
        ros_ccml.max_path_deviation = ConvertCVQfromLCMtoROS(cartesian_control_mode_limits.max_path_deviation);
        ros_ccml.max_control_force = ConvertCVQfromLCMtoROS(cartesian_control_mode_limits.max_control_force);
        ros_ccml.stop_on_max_control_force = cartesian_control_mode_limits.stop_on_max_control_force;
        return ros_ccml;
    }

    victor_hardware_interface::cartesian_control_mode_limits IIWAHardwareInterface::ConvertCartesianControlModeLimits(const victor_hardware_interface::CartesianControlModeLimits& cartesian_control_mode_limits) const
    {
        victor_hardware_interface::cartesian_control_mode_limits lcm_ccml;
        lcm_ccml.max_cartesian_velocity = ConvertCVQfromROStoLCM(cartesian_control_mode_limits.max_cartesian_velocity);
        lcm_ccml.max_path_deviation = ConvertCVQfromROStoLCM(cartesian_control_mode_limits.max_path_deviation);
        lcm_ccml.max_control_force = ConvertCVQfromROStoLCM(cartesian_control_mode_limits.max_control_force);
        lcm_ccml.stop_on_max_control_force = cartesian_control_mode_limits.stop_on_max_control_force;
        return lcm_ccml;
    }

    victor_hardware_interface::motion_command IIWAHardwareInterface::ConvertMotionCommand(const victor_hardware_interface::MotionCommand& motion_command) const
    {
        victor_hardware_interface::motion_command lcm_command;
        lcm_command.cartesian_pose = ConvertPosefromROStoLCM(motion_command.cartesian_pose);
        lcm_command.joint_position = ConvertJVQfromROStoLCM(motion_command.joint_position);
        lcm_command.joint_velocity = ConvertJVQfromROStoLCM(motion_command.joint_velocity);
        lcm_command.control_mode = (int8_t)motion_command.control_mode;
        lcm_command.timestamp = motion_command.header.stamp.toSec();
        return lcm_command;
    }

    victor_hardware_interface::MotionStatus IIWAHardwareInterface::ConvertMotionStatus(const victor_hardware_interface::motion_status& motion_status) const
    {
        victor_hardware_interface::MotionStatus ros_status;
        ros_status.commanded_cartesian_pose = ConvertPosefromLCMtoROS(motion_status.commanded_cartesian_pose);
        ros_status.commanded_cartesian_pose_raw = ConvertCVQfromLCMtoROS(motion_status.commanded_cartesian_pose_raw);
        ros_status.commanded_joint_position = ConvertJVQfromLCMtoROS(motion_status.commanded_joint_position);
        ros_status.estimated_external_torque = ConvertJVQfromLCMtoROS(motion_status.estimated_external_torque);
        ros_status.estimated_external_wrench = ConvertCVQfromLCMtoROS(motion_status.estimated_external_wrench);
        ros_status.measured_cartesian_pose = ConvertPosefromLCMtoROS(motion_status.measured_cartesian_pose);
        ros_status.measured_cartesian_pose_raw = ConvertCVQfromLCMtoROS(motion_status.measured_cartesian_pose_raw);
        ros_status.measured_joint_position = ConvertJVQfromLCMtoROS(motion_status.measured_joint_position);
        ros_status.measured_joint_torque = ConvertJVQfromLCMtoROS(motion_status.measured_joint_torque);
        ros_status.measured_joint_velocity = ConvertJVQfromLCMtoROS(motion_status.measured_joint_velocity);
        ros_status.active_control_mode = (uint8_t)motion_status.active_control_mode;
        ros_status.header.stamp = ros::Time(motion_status.timestamp);
        return ros_status;
    }

    victor_hardware_interface::control_mode_command IIWAHardwareInterface::ConvertControlModeCommand(const victor_hardware_interface::ControlModeCommand& control_mode_command) const
    {
        victor_hardware_interface::control_mode_command lcm_command;
        lcm_command.cartesian_control_mode_limits = ConvertCartesianControlModeLimits(control_mode_command.cartesian_control_mode_limits);
        lcm_command.cartesian_impedance_params = ConvertCartesianImpedanceParameters(control_mode_command.cartesian_impedance_params);
        lcm_command.joint_impedance_params = ConvertJointImpedanceParameters(control_mode_command.joint_impedance_params);
        lcm_command.joint_path_execution_params = ConvertJointPathExecutionParameters(control_mode_command.joint_path_execution_params);
        lcm_command.cartesian_path_execution_params = ConvertCartesianPathExecutionParameters(control_mode_command.cartesian_path_execution_params);
        lcm_command.control_mode = (int8_t)control_mode_command.control_mode;
        lcm_command.timestamp = control_mode_command.header.stamp.toSec();
        return lcm_command;
    }

    victor_hardware_interface::ControlModeStatus IIWAHardwareInterface::ConvertControlModeStatus(const victor_hardware_interface::control_mode_status& control_mode_status) const
    {
        victor_hardware_interface::ControlModeStatus ros_status;
        ros_status.cartesian_control_mode_limits = ConvertCartesianControlModeLimits(control_mode_status.cartesian_control_mode_limits);
        ros_status.cartesian_impedance_params = ConvertCartesianImpedanceParameters(control_mode_status.cartesian_impedance_params);
        ros_status.joint_impedance_params = ConvertJointImpedanceParameters(control_mode_status.joint_impedance_params);
        ros_status.joint_path_execution_params = ConvertJointPathExecutionParameters(control_mode_status.joint_path_execution_params);
        ros_status.cartesian_path_execution_params = ConvertCartesianPathExecutionParameters(control_mode_status.cartesian_path_execution_params);
        ros_status.active_control_mode = (uint8_t)control_mode_status.active_control_mode;
        ros_status.header.stamp = ros::Time(control_mode_status.timestamp);
        return ros_status;
    }

    bool IIWAHardwareInterface::SendMotionCommandMessage(const victor_hardware_interface::MotionCommand& command)
    {
        const victor_hardware_interface::motion_command lcm_command = ConvertMotionCommand(command);
        const int ret = send_lcm_ptr_->publish(motion_command_channel_name_, &lcm_command);
        if (ret == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool IIWAHardwareInterface::SendControlModeCommandMessage(const victor_hardware_interface::ControlModeCommand& command)
    {
        const victor_hardware_interface::control_mode_command lcm_command = ConvertControlModeCommand(command);
        const int ret = send_lcm_ptr_->publish(control_mode_command_channel_name_, &lcm_command);
        if (ret == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void IIWAHardwareInterface::InternalMotionStatusLCMCallback(const lcm::ReceiveBuffer* buffer, const std::string& channel, const victor_hardware_interface::motion_status* status_msg)
    {
        UNUSED(buffer);
        UNUSED(channel);
        const victor_hardware_interface::MotionStatus ros_status = ConvertMotionStatus(*status_msg);
        motion_status_callback_fn_(ros_status);
    }

    void IIWAHardwareInterface::InternalControlModeStatusLCMCallback(const lcm::ReceiveBuffer* buffer, const std::string& channel, const victor_hardware_interface::control_mode_status* status_msg)
    {
        UNUSED(buffer);
        UNUSED(channel);
        const victor_hardware_interface::ControlModeStatus ros_status = ConvertControlModeStatus(*status_msg);
        control_mode_status_callback_fn_(ros_status);
    }
}
