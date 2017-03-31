#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <atomic>
#include <thread>
#include <chrono>
#include <string>
#include <iostream>
#include <arc_utilities/arc_helpers.hpp>
#include <wiktor_hardware_interface/iiwa_hardware_interface.hpp>
#include <wiktor_hardware_interface/robotiq_3finger_hardware_interface.hpp>
// ROS message headers
#include <wiktor_hardware_interface/ControlModeCommand.h>
#include <wiktor_hardware_interface/ControlModeStatus.h>
#include <wiktor_hardware_interface/MotionCommand.h>
#include <wiktor_hardware_interface/MotionStatus.h>
#include <wiktor_hardware_interface/Robotiq3FingerCommand.h>
#include <wiktor_hardware_interface/Robotiq3FingerStatus.h>
#include <wiktor_hardware_interface/SetControlMode.h>
#include <wiktor_hardware_interface/GetControlMode.h>
// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
// LCM
#include <lcm/lcm-cpp.hpp>

#define RTT_CHECK_COUNT 10000

class RightArmNode
{
protected:

    ros::NodeHandle nh_;
    ros::Publisher motion_status_pub_;
    ros::Publisher gripper_status_pub_;
    ros::Subscriber motion_command_sub_;
    ros::Subscriber gripper_command_sub_;
    ros::ServiceServer set_control_mode_server_;
    ros::ServiceServer get_control_mode_server_;
    ros::CallbackQueue ros_callback_queue_;
    std::thread ros_callback_thread_;

    bool has_active_control_mode_;
    wiktor_hardware_interface::ControlModeStatus active_control_mode_;
    std::atomic<bool> setting_control_mode_;

    std::shared_ptr<lcm::LCM> send_lcm_ptr_;
    std::shared_ptr<lcm::LCM> recv_lcm_ptr_;
    std::unique_ptr<iiwa_hardware_interface::IIWAHardwareInterface> iiwa_ptr_;
    std::unique_ptr<robotiq_3finger_hardware_interface::Robotiq3FingerHardwareInterface> robotiq_ptr_;

public:

    RightArmNode(ros::NodeHandle& nh, const std::string& motion_command_topic, const std::string& motion_status_topic, const std::string& get_control_mode_service, const std::string& set_control_mode_service, const std::string& gripper_command_topic, const std::string& gripper_status_topic, const std::shared_ptr<lcm::LCM>& send_lcm_ptr, const std::shared_ptr<lcm::LCM>& recv_lcm_ptr, const std::string& motion_command_channel, const std::string& motion_status_channel, const std::string& control_mode_command_channel, const std::string& control_mode_status_channel, const std::string& gripper_command_channel, const std::string& gripper_status_channel) : nh_(nh), send_lcm_ptr_(send_lcm_ptr), recv_lcm_ptr_(recv_lcm_ptr)
    {
        setting_control_mode_.store(false);
        nh_.setCallbackQueue(&ros_callback_queue_);
        // Set up IIWA LCM interface
        std::function<void(const wiktor_hardware_interface::MotionStatus&)> motion_status_callback_fn = [&] (const wiktor_hardware_interface::MotionStatus& motion_status) { return MotionStatusCallback(motion_status); };
        std::function<void(const wiktor_hardware_interface::ControlModeStatus&)> control_mode_status_callback_fn = [&] (const wiktor_hardware_interface::ControlModeStatus& control_mode_status) { return ControlModeStatusCallback(control_mode_status); };
        iiwa_ptr_ = std::unique_ptr<iiwa_hardware_interface::IIWAHardwareInterface>(new iiwa_hardware_interface::IIWAHardwareInterface(send_lcm_ptr_, recv_lcm_ptr_, motion_command_channel, motion_status_channel, motion_status_callback_fn, control_mode_command_channel, control_mode_status_channel, control_mode_status_callback_fn));
        // Set up Robotiq LCM interface
        std::function<void(const wiktor_hardware_interface::Robotiq3FingerStatus&)> gripper_status_callback_fn = [&] (const wiktor_hardware_interface::Robotiq3FingerStatus& gripper_status) { return GripperStatusCallback(gripper_status); };
        robotiq_ptr_ = std::unique_ptr<robotiq_3finger_hardware_interface::Robotiq3FingerHardwareInterface>(new robotiq_3finger_hardware_interface::Robotiq3FingerHardwareInterface(send_lcm_ptr_, recv_lcm_ptr_, gripper_command_channel, gripper_status_channel, gripper_status_callback_fn));
        // Set up ROS interfaces
        motion_status_pub_ = nh_.advertise<wiktor_hardware_interface::MotionStatus>(motion_status_topic, 1, false);
        gripper_status_pub_ = nh_.advertise<wiktor_hardware_interface::Robotiq3FingerStatus>(gripper_status_topic, 1, false);
        motion_command_sub_ = nh_.subscribe(motion_command_topic, 1, &RightArmNode::MotionCommandROSCallback, this);
        gripper_command_sub_ = nh_.subscribe(gripper_command_topic, 1, &RightArmNode::GripperCommandROSCallback, this);;
        set_control_mode_server_ = nh_.advertiseService(set_control_mode_service, &RightArmNode::SetControlModeCallback, this);
        get_control_mode_server_ = nh_.advertiseService(get_control_mode_service, &RightArmNode::GetControlModeCallback, this);
        // Start ROS thread
        ros_callback_thread_ = std::thread(std::bind(&RightArmNode::ROSCallbackThread, this));
    }

    void ROSCallbackThread()
    {
        const double timeout = 0.001;
        while (nh_.ok())
        {
            ros_callback_queue_.callAvailable(ros::WallDuration(timeout));
        }
    }

    void LCMLoop()
    {
        // Run LCM
        bool lcm_ok = true;
        while (ros::ok() && lcm_ok)
        {
            bool lcm_running = true;
            while (lcm_running)
            {
                const int ret = recv_lcm_ptr_->handleTimeout(1);
                if (ret > 0)
                {
                    lcm_running = true;
                }
                else if (ret == 0)
                {
                    lcm_running = false;
                }
                else
                {
                    lcm_running = false;
                    lcm_ok = false;
                    ROS_ERROR_STREAM_NAMED(ros::this_node::getName(), "LCM error " << ret);
                }
            }
        }
    }

    static inline bool JVQMatch(const wiktor_hardware_interface::JointValueQuantity& jvq1, const wiktor_hardware_interface::JointValueQuantity& jvq2)
    {
        if (jvq1.joint_1 != jvq2.joint_1)
        {
            return false;
        }
        else if (jvq1.joint_2 != jvq2.joint_2)
        {
            return false;
        }
        else if (jvq1.joint_3 != jvq2.joint_3)
        {
            return false;
        }
        else if (jvq1.joint_4 != jvq2.joint_4)
        {
            return false;
        }
        else if (jvq1.joint_5 != jvq2.joint_5)
        {
            return false;
        }
        else if (jvq1.joint_6 != jvq2.joint_6)
        {
            return false;
        }
        else if (jvq1.joint_7 != jvq2.joint_7)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    static inline bool CVQMatch(const wiktor_hardware_interface::CartesianValueQuantity& cvq1, const wiktor_hardware_interface::CartesianValueQuantity& cvq2)
    {
        if (cvq1.x != cvq2.x)
        {
            return false;
        }
        else if (cvq1.y != cvq2.y)
        {
            return false;
        }
        else if (cvq1.z != cvq2.z)
        {
            return false;
        }
        else if (cvq1.a != cvq2.a)
        {
            return false;
        }
        else if (cvq1.b != cvq2.b)
        {
            return false;
        }
        else if (cvq1.c != cvq2.c)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    static inline bool PExPMatch(const wiktor_hardware_interface::PathExecutionParameters& pexp1, const wiktor_hardware_interface::PathExecutionParameters& pexp2)
    {
        if (pexp1.joint_relative_acceleration != pexp2.joint_relative_acceleration)
        {
            return false;
        }
        else if (pexp1.joint_relative_velocity != pexp2.joint_relative_velocity)
        {
            return false;
        }
        else if (pexp1.override_joint_acceleration != pexp2.override_joint_acceleration)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    bool CheckControlModeCommandAndStatusMatch(const wiktor_hardware_interface::ControlModeCommand& command, const wiktor_hardware_interface::ControlModeStatus& status) const
    {
        const bool cdmatch = CVQMatch(command.cartesian_impedance_params.cartesian_damping, status.cartesian_impedance_params.cartesian_damping);
        const bool ndmatch = (command.cartesian_impedance_params.nullspace_damping == status.cartesian_impedance_params.nullspace_damping);
        const bool csmatch = CVQMatch(command.cartesian_impedance_params.cartesian_stiffness, status.cartesian_impedance_params.cartesian_stiffness);
        const bool nsmatch = (command.cartesian_impedance_params.nullspace_stiffness == status.cartesian_impedance_params.nullspace_stiffness);
        const bool jdmatch = JVQMatch(command.joint_impedance_params.joint_damping, status.joint_impedance_params.joint_damping);
        const bool jsmatch = JVQMatch(command.joint_impedance_params.joint_stiffness, status.joint_impedance_params.joint_stiffness);
        const bool pexpmatch = PExPMatch(command.path_execution_params, status.path_execution_params);
        const bool cmmatch = (command.control_mode == status.active_control_mode);
        if (cdmatch && ndmatch && csmatch && nsmatch && jdmatch && jsmatch && pexpmatch && cmmatch)
        {
            return true;
        }
        else
        {
            return true;
        }
    }

    std::pair<bool, std::string> SafetyCheckControlMode(const wiktor_hardware_interface::ControlModeCommand& control_mode) const
    {
        UNUSED(control_mode);
        return std::make_pair(false, "");
    }

    bool SetControlModeCallback(wiktor_hardware_interface::SetControlMode::Request& req, wiktor_hardware_interface::SetControlMode::Response& res)
    {
        if (setting_control_mode_.load())
        {
            res.success = false;
            res.message = "Cannot set control mode, control mode setting already in progress";
            return true;
        }
        const std::pair<bool, std::string> safety_check = SafetyCheckControlMode(req.new_control_mode);
        if (safety_check.first)
        {
            iiwa_ptr_->SendControlModeCommandMessage(req.new_control_mode);
            setting_control_mode_.store(true);
            while (setting_control_mode_.load())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            if (CheckControlModeCommandAndStatusMatch(req.new_control_mode, active_control_mode_))
            {
                res.success = true;
                res.message = "Control mode set successfully";
            }
            else
            {
                res.success = false;
                res.message = "Control mode could not be set in Sunrise";
            }
        }
        else
        {
            res.success = false;
            res.message = safety_check.second;
        }
        return true;
    }

    bool GetControlModeCallback(wiktor_hardware_interface::GetControlMode::Request& req, wiktor_hardware_interface::GetControlMode::Response& res)
    {
        UNUSED(req);
        res.has_active_control_mode = has_active_control_mode_;
        res.active_control_mode = active_control_mode_;
        return true;
    }

    bool SafetyCheckPositions(const wiktor_hardware_interface::JointValueQuantity& positions) const
    {
        UNUSED(positions);
        return true;
    }

    bool SafetyCheckPositionsVelocities(const wiktor_hardware_interface::JointValueQuantity& positions, const wiktor_hardware_interface::JointValueQuantity& velocities) const
    {
        UNUSED(positions);
        UNUSED(velocities);
        return true;
    }

    bool SafetyCheckCartesianPose(const wiktor_hardware_interface::CartesianValueQuantity& pose) const
    {
        UNUSED(pose);
        return true;
    }

    bool SafetyCheckMotionCommand(const wiktor_hardware_interface::MotionCommand& command) const
    {
        if (has_active_control_mode_)
        {
            const uint8_t active_control_type = active_control_mode_.active_control_mode;
            const uint8_t command_motion_type = command.command_type;
            if (active_control_type == wiktor_hardware_interface::ControlModeCommand::JOINT_POSITION)
            {
                if (command_motion_type == wiktor_hardware_interface::MotionCommand::JOINT_POSITION)
                {
                    return SafetyCheckPositions(command.joint_position);
                }
                else if (command_motion_type == wiktor_hardware_interface::MotionCommand::JOINT_POSITION_VELOCITY)
                {
                    return SafetyCheckPositionsVelocities(command.joint_position, command.joint_velocity);
                }
                else
                {
                    return false;
                }
            }
            else if (active_control_type == wiktor_hardware_interface::ControlModeCommand::JOINT_IMPEDANCE)
            {
                if (command_motion_type == wiktor_hardware_interface::MotionCommand::JOINT_POSITION)
                {
                    return SafetyCheckPositions(command.joint_position);
                }
                else
                {
                    return false;
                }
            }
            else if (active_control_type == wiktor_hardware_interface::ControlModeCommand::CARTESIAN_POSE || active_control_type == wiktor_hardware_interface::ControlModeCommand::CARTESIAN_IMPEDANCE)
            {
                if (command_motion_type == wiktor_hardware_interface::MotionCommand::CARTESIAN_POSE)
                {
                    return SafetyCheckCartesianPose(command.cartesian_pose);
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    void MotionCommandROSCallback(wiktor_hardware_interface::MotionCommand command)
    {
        if (SafetyCheckMotionCommand(command))
        {
            iiwa_ptr_->SendMotionCommandMessage(command);
        }
    }

    bool SafetyCheckFingerCommand(const wiktor_hardware_interface::Robotiq3FingerActuatorCommand& command) const
    {
        if (command.position > 1.0 || command.position < 0.0)
        {
            return false;
        }
        else if (command.force > 1.0 || command.force < 0.0)
        {
            return false;
        }
        else if (command.speed > 1.0 || command.speed < 0.0)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    bool SafetyCheckGripperCommand(const wiktor_hardware_interface::Robotiq3FingerCommand& command) const
    {
        const bool ac = SafetyCheckFingerCommand(command.finger_a_command);
        const bool bc = SafetyCheckFingerCommand(command.finger_b_command);
        const bool cc = SafetyCheckFingerCommand(command.finger_c_command);
        const bool sc = SafetyCheckFingerCommand(command.scissor_command);
        if (ac && bc && cc && sc)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void GripperCommandROSCallback(wiktor_hardware_interface::Robotiq3FingerCommand command)
    {
        if (SafetyCheckGripperCommand(command))
        {
            robotiq_ptr_->SendCommandMessage(command);
        }
    }

    void MotionStatusCallback(const wiktor_hardware_interface::MotionStatus& motion_status)
    {
        motion_status_pub_.publish(motion_status);
    }

    void ControlModeStatusCallback(const wiktor_hardware_interface::ControlModeStatus& control_mode_status)
    {
        if (setting_control_mode_.load())
        {
            has_active_control_mode_ = true;
            active_control_mode_ = control_mode_status;
            setting_control_mode_.store(false);
        }
        else
        {
            ROS_ERROR_NAMED(ros::this_node::getName(), "Got a control mode status message we did not expect, dropping!");
        }
    }

    void GripperStatusCallback(const wiktor_hardware_interface::Robotiq3FingerStatus& gripper_status)
    {
        gripper_status_pub_.publish(gripper_status);
    }

};

int main(int argc, char** argv)
{
    // Default ROS topic / service names
    const std::string DEFAULT_MOTION_COMMAND_TOPIC("motion_command");
    const std::string DEFAULT_MOTION_STATUS_TOPIC("motion_status");
    const std::string DEFAULT_SET_CONTROL_MODE_SERVICE("set_control_mode_service");
    const std::string DEFAULT_GET_CONTROL_MODE_SERVICE("get_control_mode_service");
    const std::string DEFAULT_GRIPPER_COMMAND_TOPIC("gripper_command");
    const std::string DEFAULT_GRIPPER_STATUS_TOPIC("gripper_status");
    // Default LCM parameters
    const std::string DEFAULT_SEND_LCM_URL("udp://10.10.10.11:30001");
    const std::string DEFAULT_RECV_LCM_URL("udp://10.10.10.99:30000");
    const std::string DEFAULT_MOTION_COMMAND_CHANNEL("motion_command");
    const std::string DEFAULT_MOTION_STATUS_CHANNEL("motion_status");
    const std::string DEFAULT_CONTROL_MODE_COMMAND_CHANNEL("control_mode_command");
    const std::string DEFAULT_CONTROL_MODE_STATUS_CHANNEL("control_mode_status");
    const std::string DEFAULT_GRIPPER_COMMAND_CHANNEL("gripper_command");
    const std::string DEFAULT_GRIPPER_STATUS_CHANNEL("gripper_status");
    // Start ROS
    ros::init(argc, argv, "right_arm_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    // Get topic & service names
    const std::string motion_command_topic = nhp.param(std::string("motion_command_topic"), DEFAULT_MOTION_COMMAND_TOPIC);
    const std::string motion_status_topic = nhp.param(std::string("motion_status_topic"), DEFAULT_MOTION_STATUS_TOPIC);
    const std::string get_control_mode_service = nhp.param(std::string("get_control_mode_service"), DEFAULT_GET_CONTROL_MODE_SERVICE);
    const std::string set_control_mode_service = nhp.param(std::string("set_control_mode_service"), DEFAULT_SET_CONTROL_MODE_SERVICE);
    const std::string gripper_command_topic = nhp.param(std::string("gripper_command_topic"), DEFAULT_GRIPPER_COMMAND_TOPIC);
    const std::string gripper_status_topic = nhp.param(std::string("gripper_status_topic"), DEFAULT_GRIPPER_STATUS_TOPIC);
    // Get LCM params
    const std::string send_lcm_url = nhp.param(std::string("send_lcm_url"), DEFAULT_SEND_LCM_URL);
    const std::string recv_lcm_url = nhp.param(std::string("recv_lcm_url"), DEFAULT_RECV_LCM_URL);
    const std::string motion_command_channel = nhp.param(std::string("motion_command_channel"), DEFAULT_MOTION_COMMAND_CHANNEL);
    const std::string motion_status_channel = nhp.param(std::string("motion_status_channel"), DEFAULT_MOTION_STATUS_CHANNEL);
    const std::string control_mode_command_channel = nhp.param(std::string("control_mode_command_channel"), DEFAULT_CONTROL_MODE_COMMAND_CHANNEL);
    const std::string control_mode_status_channel = nhp.param(std::string("control_mode_status_channel"), DEFAULT_CONTROL_MODE_STATUS_CHANNEL);
    const std::string gripper_command_channel = nhp.param(std::string("gripper_command_channel"), DEFAULT_GRIPPER_COMMAND_CHANNEL);
    const std::string gripper_status_channel = nhp.param(std::string("gripper_status_channel"), DEFAULT_GRIPPER_STATUS_CHANNEL);
    // Start LCM
    if (send_lcm_url == recv_lcm_url)
    {
        std::shared_ptr<lcm::LCM> lcm_ptr(new lcm::LCM(send_lcm_url));
        ROS_INFO("Starting Right Arm Node with shared send/receive LCM...");
        RightArmNode node(nh, motion_command_topic, motion_status_topic, get_control_mode_service, set_control_mode_service, gripper_command_topic, gripper_status_topic, lcm_ptr, lcm_ptr, motion_command_channel, motion_status_channel, control_mode_command_channel, control_mode_status_channel, gripper_command_channel, gripper_status_channel);
        node.LCMLoop();
        return 0;
    }
    else
    {
        std::shared_ptr<lcm::LCM> send_lcm_ptr(new lcm::LCM(send_lcm_url));
        std::shared_ptr<lcm::LCM> recv_lcm_ptr(new lcm::LCM(recv_lcm_url));
        ROS_INFO("Starting Right Arm Node with separate send and receive LCM...");
        RightArmNode node(nh, motion_command_topic, motion_status_topic, get_control_mode_service, set_control_mode_service, gripper_command_topic, gripper_status_topic, send_lcm_ptr, recv_lcm_ptr, motion_command_channel, motion_status_channel, control_mode_command_channel, control_mode_status_channel, gripper_command_channel, gripper_status_channel);
        node.LCMLoop();
        return 0;
    }
}
