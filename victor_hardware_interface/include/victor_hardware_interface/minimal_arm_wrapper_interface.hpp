#include <thread>
#include <string>
#include <mutex>
#include <arc_utilities/maybe.hpp>

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
// ROS message headers
#include <victor_hardware_interface_msgs/ControlModeParameters.h>
#include <victor_hardware_interface_msgs/MotionCommand.h>
#include <victor_hardware_interface_msgs/MotionStatus.h>
#include <victor_hardware_interface_msgs/Robotiq3FingerCommand.h>
#include <victor_hardware_interface_msgs/Robotiq3FingerStatus.h>
#include <victor_hardware_interface_msgs/SetControlMode.h>
#include <victor_hardware_interface_msgs/GetControlMode.h>

// LCM
#include <lcm/lcm-cpp.hpp>

// Classes to speak to each invididual hardware element
#include <victor_hardware_interface/iiwa_hardware_interface.hpp>
#include <victor_hardware_interface/robotiq_3finger_hardware_interface.hpp>

#ifndef MINIMAL_ARM_WRAPPER_INTERFACE_HPP
#define MINIMAL_ARM_WRAPPER_INTERFACE_HPP

namespace victor_hardware_interface
{
    namespace msgs = victor_hardware_interface_msgs;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Helpers to test if two messages are equivalent
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    bool jvqEqual(const msgs::JointValueQuantity& jvq1, const msgs::JointValueQuantity& jvq2);

    bool cvqEqual(const msgs::CartesianValueQuantity& cvq1, const msgs::CartesianValueQuantity& cvq2);

    bool jointPexpEqual(const msgs::JointPathExecutionParameters& pexp1, const msgs::JointPathExecutionParameters& pexp2);

    bool cartesianPexpEqual(const msgs::CartesianPathExecutionParameters& pexp1, const msgs::CartesianPathExecutionParameters& pexp2);

    bool controlModeParamsEqual(const msgs::ControlModeParameters& params1, const msgs::ControlModeParameters& params2);


    /**
     * @brief The MinimalArmWrapperInterface class
     */
    class MinimalArmWrapperInterface
    {
    public:

        MinimalArmWrapperInterface(
                ros::NodeHandle& nh,
                const std::shared_ptr<lcm::LCM>& send_lcm_ptr,
                const std::shared_ptr<lcm::LCM>& recv_lcm_ptr,
                const std::string& cartesian_control_frame,
                const double set_control_mode_timeout,
                // ROS Topics
                const std::string& motion_command_topic,
                const std::string& motion_status_topic,
                const std::string& control_mode_status_topic,
                const std::string& get_control_mode_service,
                const std::string& set_control_mode_service,
                const std::string& gripper_command_topic,
                const std::string& gripper_status_topic,
                // LCM channels
                const std::string& motion_command_channel,
                const std::string& motion_status_channel,
                const std::string& control_mode_command_channel,
                const std::string& control_mode_status_channel,
                const std::string& gripper_command_channel,
                const std::string& gripper_status_channel);

        // The one function called externally
        void spin();

    protected:

        constexpr static double ROS_SPIN_PERIOD = 0.001;   // measured in seconds
        constexpr static int LCM_HANDLE_TIMEOUT = 1;       // measured in milliseconds

        ros::NodeHandle nh_;
        const std::string cartesian_control_frame_;
        ros::Publisher motion_status_pub_;
        ros::Publisher control_mode_status_pub_;
        ros::Publisher gripper_status_pub_;
        ros::Subscriber motion_command_sub_;
        ros::Subscriber gripper_command_sub_;
        ros::ServiceServer set_control_mode_server_;
        ros::ServiceServer get_control_mode_server_;
        ros::CallbackQueue ros_callback_queue_;

        mutable std::mutex control_mode_status_mutex_;
        Maybe::Maybe<msgs::ControlModeParameters> active_control_mode_;
        const double set_control_mode_timeout_; // measured in seconds

        std::shared_ptr<lcm::LCM> send_lcm_ptr_;
        std::shared_ptr<lcm::LCM> recv_lcm_ptr_;
        std::unique_ptr<IIWAHardwareInterface> iiwa_ptr_;
        std::unique_ptr<Robotiq3FingerHardwareInterface> robotiq_ptr_;


        // Internal helper function used to process ROS callbacks
        void rosSpinThread();

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Get/Set Control Mode functionality
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //// Static helpers to check parameters for being in valid ranges for the control mode /////////////////////////

        static std::pair<bool, std::string> validateJointPathExecutionParams(const msgs::JointPathExecutionParameters& params);

        static std::pair<bool, std::string> validateCartesianPathExecutionParams(const msgs::CartesianPathExecutionParameters& params);

        static std::pair<bool, std::string> validateJointImpedanceParams(const msgs::JointImpedanceParameters& params);

        static std::pair<bool, std::string> validateCartesianImpedanceParams(const msgs::CartesianImpedanceParameters& params);

        static std::pair<bool, std::string> validateCartesianControlModeLimits(const msgs::CartesianControlModeLimits& params);

        static std::pair<bool, std::string> validateControlMode(const msgs::ControlModeParameters& params);

        //// ROS Callbacks to get and set the control mode as service calls ////////////////////////////////////////////

        bool setControlModeCallback(msgs::SetControlMode::Request& req, msgs::SetControlMode::Response& res);

        bool getControlModeCallback(msgs::GetControlMode::Request& req, msgs::GetControlMode::Response& res);

        /*
         * Callback function used by the LCM subsystem when a control_mode_status message is received. Caches the value
         * in active_control_mode_ for use by setControlModeCallback(...) and getControlModeCallback(...)
         */
        void controlModeStatusLCMCallback(const msgs::ControlModeParameters& control_mode_status);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Arm movement/control and feedback functionality
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //// Static helpers to check parameters for being in valid ranges for the motion command ///////////////////////

        static std::pair<bool, std::string> validateJointPositions(const msgs::JointValueQuantity& positions);

        std::pair<bool, std::string> validateCartesianPose(const geometry_msgs::Pose& pose, const std::string& frame) const;

        std::pair<bool, std::string> validateMotionCommand(const msgs::MotionCommand& command) const;

        /*
         * ROS callback to parse a arm motion command, and pass it along to the LCM subsystem
         */
        void motionCommandROSCallback(const msgs::MotionCommand& command);

        /*
         * LCM callback used by the LCM subsystem when a LCM status message is recieved. Republishes the motion status
         * on the correct ROS topic
         */
        void motionStatusLCMCallback(const msgs::MotionStatus& motion_status);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Gripper movement/control and feedback functionality
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //// Static helpers to check parameters for being in valid ranges for the gripper command //////////////////////

        static std::pair<bool, std::string> validateFingerCommand(const msgs::Robotiq3FingerActuatorCommand& command);

        static std::pair<bool, std::string> validateGripperCommand(const msgs::Robotiq3FingerCommand& command);

        /*
         * ROS callback to parse a gripper motion command, and pass it along to the LCM subsystem
         */
        void gripperCommandROSCallback(const msgs::Robotiq3FingerCommand& command);

        /*
         * LCM callback used by the LCM subsystem when a LCM status message is recieved. Republishes the motion status
         * on the correct ROS topic
         */
        void gripperStatusLCMCallback(const msgs::Robotiq3FingerStatus& gripper_status);

    };
}

#endif // MINIMAL_ARM_WRAPPER_INTERFACE_HPP
