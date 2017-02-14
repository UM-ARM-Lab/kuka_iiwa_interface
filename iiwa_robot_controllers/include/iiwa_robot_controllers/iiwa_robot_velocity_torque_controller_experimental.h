#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <random>
#include <Eigen/Geometry>
#include <time.h>
#include <chrono>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <iiwa_robot_controllers/VelocityCommand.h>
#include <iiwa_robot_controllers/TorqueCommand.h>
#include <iiwa_robot_controllers/iiwa_robot_config.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <linux_fri_client/friConnectionIf.h>
#include <linux_fri_client/friClientData.h>
#include <linux_fri_client/FRIMessages.pb.h>

namespace iiwa_robot_controllers
{
    class RobotInterface
    {
    protected:

        std::shared_ptr<KUKA::FRI::IConnection> connection_ptr_;

    public:

        RobotInterface(const std::shared_ptr<KUKA::FRI::IConnection>& connection_ptr) : connection_ptr_(connection_ptr) {}

        ~RobotInterface()
        {
            if (connection_ptr_->isOpen())
            {
                connection_ptr_->close();
            }
        }

        bool GetCurrentState(std::vector<double>& current_config, std::vector<double>& current_velocities, std::vector<double>& current_accelerations) const
        {
            // Make sure the out params have enough space, and zero first if so
            if (current_config.size() != 7)
            {
                current_config.resize(7, 0.0);;
            }
            else
            {
                for (size_t idx = 0; idx < 7; idx++)
                {
                    current_config[idx] = 0.0;
                }
            }
            if (current_velocities.size() != 7)
            {
                current_velocities.resize(7, 0.0);;
            }
            else
            {
                for (size_t idx = 0; idx < 7; idx++)
                {
                    current_velocities[idx] = 0.0;
                }
            }
            if (current_accelerations.size() != 7)
            {
                current_accelerations.resize(7, 0.0);;
            }
            else
            {
                for (size_t idx = 0; idx < 7; idx++)
                {
                    current_accelerations[idx] = 0.0;
                }
            }
            // Copy data over
            ;
            // Return status
            return false;
        }

        bool CommandTorques(const std::vector<double>& torque_command)
        {
            if (torque_command.size() == 7)
            {
                // Check each value
                for (size_t idx = 0; idx < 7; idx++)
                {
                    const double command_val = torque_command[idx];
                    if (std::isnan(command_val) || std::isinf(command_val))
                    {
                        return false;
                    }
                }
                // Command
                ;
                return false;
            }
            else
            {
                return false;
            }
        }

        bool Start(const int connection_port, const std::string& robot_address)
        {
            if (connection_ptr_->isOpen())
            {
                return true;
            }
            else
            {
                return connection_ptr_->open(connection_port, robot_address.c_str());
            }
        }

        bool Stop()
        {
            return false;
        }
    };

    class IIWARobotVelocityTorqueController
    {
    protected:

        enum ARM_TYPE {LEFT_ARM, RIGHT_ARM};

        ARM_TYPE arm_;

        std::vector<std::string> joint_names_;
        std::map<std::string, JointLimits> joint_limits_;
        std::map<std::string, PIDParams> joint_controller_params_;

        KDL::Chain chain_;
        std::shared_ptr<KDL::ChainIdSolver_RNE> id_solver_;

        std::shared_ptr<RobotInterface> robot_interface_ptr_;

        std::atomic<bool> target_velocities_valid_;
        std::mutex target_velocities_lock_;
        std::vector<double> target_velocities_;

        std::vector<double> velocity_error_integrals_;
        std::vector<double> last_velocity_errors_;

        ros::NodeHandle nh_;
        ros::CallbackQueue ros_callback_queue_;
        std::thread ros_callback_thread_;
        ros::Subscriber velocity_target_sub_;
        ros::ServiceServer abort_server_;

    public:

        IIWARobotVelocityTorqueController(ros::NodeHandle& nh,
                                            const std::string& velocity_command_topic,
                                            const std::string& abort_service,
                                            const std::string& xml_model_string,
                                            const bool model_gravity,
                                            const std::map<std::string, JointLimits> joint_limits,
                                            const std::map<std::string, PIDParams> joint_controller_params,
                                            const std::shared_ptr<RobotInterface>& robot_interface_ptr) : robot_interface_ptr_(robot_interface_ptr), nh_(nh)
        {
            target_velocities_valid_.store(false);
            target_velocities_ = std::vector<double>();
            assert(SetsEqual(arc_helpers::GetKeys(joint_limits), arc_helpers::GetKeys(joint_controller_params)));
            joint_names_ = arc_helpers::GetKeys(joint_limits);
            velocity_error_integrals_ = std::vector<double>(joint_names_.size(), 0.0);
            last_velocity_errors_ = std::vector<double>(joint_names_.size(), 0.0);
            joint_limits_ = joint_limits;
            joint_controller_params_ = joint_controller_params;
            // Setup KDL chain
            KDL::Tree tree;
            if (!kdl_parser::treeFromString(xml_model_string, tree))
            {
                ROS_ERROR("Could not initialize tree object");
                assert(false);
            }
            const std::string root_name = "link_0";
            const std::string tip_name = "link_ee";
            if (!tree.getChain(root_name, tip_name, chain_))
            {
                ROS_ERROR("Could not initialize chain object for root_name %s and tip_name %s", root_name.c_str(), tip_name.c_str());
                assert(false);
            }
            // Setup ID solver
            const KDL::Vector gravity = model_gravity ? KDL::Vector(0.0, 0.0, -9.81) : KDL::Vector(0.0, 0.0, 0.0);
            id_solver_ = std::shared_ptr<KDL::ChainIdSolver_RNE>(new KDL::ChainIdSolver_RNE(chain_, gravity));
            // World ROS interface is handled in its own thread
            ros::SubscribeOptions velocity_target_subscribe_options = ros::SubscribeOptions::create<iiwa_robot_controllers::VelocityCommand>(velocity_command_topic, 1, boost::bind(&IIWARobotVelocityTorqueController::VelocityTargetCallback, this, _1), ros::VoidPtr(), &ros_callback_queue_);
            ros::AdvertiseServiceOptions abort_service_options = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(abort_service, boost::bind(&IIWARobotVelocityTorqueController::AbortCB, this, _1, _2), ros::VoidPtr(), &ros_callback_queue_);
            velocity_target_sub_ = nh_.subscribe(velocity_target_subscribe_options);
            abort_server_ = nh_.advertiseService(abort_service_options);
            // Spin up the callback helper thread.
            ros_callback_thread_ = std::thread(std::bind(&IIWARobotVelocityTorqueController::ROSCallbackThread, this));
        }

        void ROSCallbackThread()
        {
            const double timeout = 0.001;
            while (nh_.ok())
            {
                ros_callback_queue_.callAvailable(ros::WallDuration(timeout));
            }
        }

        void Loop(const double control_rate)
        {
            std::vector<double> current_config;
            std::vector<double> current_velocities;
            std::vector<double> current_accelerations;
            const double control_interval = 1.0 / control_rate;
            ros::Rate spin_rate(control_rate);
            //const bool started = robot_interface_ptr_->Start();
            assert(started);
            while (ros::ok())
            {
                const bool current_config_valid = robot_interface_ptr_->GetCurrentState(current_config, current_velocities, current_accelerations);
                if (target_velocities_valid_.load() && current_config_valid)
                {
                    target_velocities_lock_.lock();
                    const std::vector<double> target_velocities = target_velocities_;
                    target_velocities_lock_.unlock();
                    const std::vector<double> raw_torque_command = ComputeRawNextStep(current_config, current_velocities, current_accelerations, target_velocities, control_interval);
                    //std::cout << "Raw torque command: " << PrettyPrint::PrettyPrint(raw_torque_command) << std::endl;
                    const std::vector<double> limited_torque_command = EnforceTorqueLimits(raw_torque_command, joint_names_, joint_limits_);
                    //std::cout << "Limited torque command: " << PrettyPrint::PrettyPrint(limited_torque_command) << std::endl;
                    const std::vector<double> torque_command = EnforcePositionSafetyLimits(current_config, limited_torque_command, joint_names_, joint_limits_);
                    //std::cout << "Final torque command: " << PrettyPrint::PrettyPrint(torque_command) << std::endl;
                    robot_interface_ptr_->CommandTorques(torque_command);
                }
                // Spin
                spin_rate.sleep();
            }
            robot_interface_ptr_->Stop();
        }

        inline bool AbortCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
        {
            UNUSED(req);
            UNUSED(res);
            ROS_INFO("Aborting config target");
            target_velocities_valid_.store(false);
            velocity_error_integrals_ = std::vector<double>(joint_names_.size(), 0.0);
            last_velocity_errors_ = std::vector<double>(joint_names_.size(), 0.0);
            return true;
        }

        inline static double ClampValue(const double value, const double lower_limit, const double upper_limit)
        {
            return std::max(lower_limit, std::min(upper_limit, value));
        }

        inline std::vector<double> ComputeVelocityError(const std::vector<double>& current_velocities, const std::vector<double>& target_velocities) const
        {
            const std::vector<double> velocity_error = EigenHelpers::Sub(target_velocities, current_velocities);
            return velocity_error;
        }

        inline std::vector<double> ComputeNewVelocityErrorIntegrals(const std::vector<double>& raw_velocity_error_integrals, const std::vector<std::string>& joint_names, const std::map<std::string, PIDParams>& joint_params) const
        {
            assert(raw_velocity_error_integrals.size() == joint_names.size());
            std::vector<double> new_velocity_error_integrals(raw_velocity_error_integrals.size(), 0.0);
            for (size_t idx = 0; idx < new_velocity_error_integrals.size(); idx++)
            {
                const double joint_velocity_error_integral = raw_velocity_error_integrals[idx];
                const std::string& joint_name = joint_names[idx];
                const auto found_itr = joint_params.find(joint_name);
                assert(found_itr != joint_params.end());
                const double i_clamp = found_itr->second.Iclamp();
                const double clamped_joint_velocity_error_integral = ClampValue(joint_velocity_error_integral, -i_clamp, i_clamp);
                new_velocity_error_integrals[idx] = clamped_joint_velocity_error_integral;
            }
            return new_velocity_error_integrals;
        }

        inline std::vector<double> ComputeFeedbackTerms(const std::vector<double>& current_velocities, const std::vector<double>& target_velocities, const double time_interval)
        {
            assert(joint_names_.size() == current_velocities.size());
            assert(joint_names_.size() == target_velocities.size());
            // Compute the pose error in our 'world frame'
            const std::vector<double> velocity_errors = ComputeVelocityError(current_velocities, target_velocities);
            // Compute the integral of pose error & update the stored value
            const std::vector<double> velocity_error_integrals_update = EigenHelpers::Add(EigenHelpers::Multiply(last_velocity_errors_, 0.5), EigenHelpers::Multiply(velocity_errors, 0.0));
            velocity_error_integrals_ = ComputeNewVelocityErrorIntegrals(EigenHelpers::Add(velocity_error_integrals_, velocity_error_integrals_update), joint_names_, joint_controller_params_);
            // Compute the derivative of pose error
            const std::vector<double> velocity_error_derivatives = EigenHelpers::Divide(EigenHelpers::Sub(velocity_errors, last_velocity_errors_), time_interval);
            // Update the stored pose error
            last_velocity_errors_ = velocity_errors;
            // Compute Feedback terms
            std::vector<double> feedback_terms(velocity_errors.size(), 0.0);
            for (size_t idx = 0; idx < feedback_terms.size(); idx++)
            {
                const double joint_velocity_error = velocity_errors[idx];
                const double joint_velocity_error_integral = velocity_error_integrals_[idx];
                const double joint_velocity_error_derivative = velocity_error_derivatives[idx];
                const std::string& joint_name = joint_names_[idx];
                const auto found_itr = joint_controller_params_.find(joint_name);
                assert(found_itr != joint_controller_params_.end());
                const PIDParams& params = found_itr->second;
                const double p_term = joint_velocity_error * params.Kp();
                const double i_term = joint_velocity_error_integral * params.Ki();
                const double d_term = joint_velocity_error_derivative * params.Kd();
    //            if (std::abs(d_term) > std::abs(p_term))
    //            {
    //                std::cout << "Dterm > Pterm!!!" << std::endl;
    //            }
    //            const double safe_d_term = ClampValue(d_term, -std::abs(p_term), std::abs(p_term));
                const double feedback_term = p_term + i_term + d_term;
                feedback_terms[idx] = feedback_term;
            }
            return feedback_terms;
        }

        inline std::vector<double> ComputeFeedForwardTerms(const std::vector<double>& current_config, const std::vector<double>& current_velocities, const std::vector<double>& current_accelerations, const std::vector<double>& target_velocities)
        {
            assert(joint_names_.size() == current_config.size());
            assert(joint_names_.size() == current_velocities.size());
            assert(joint_names_.size() == current_accelerations.size());
            assert(joint_names_.size() == target_velocities.size());
            // Zero external wrench
            KDL::Wrenches external_wrench(chain_.getNrOfSegments(), KDL::Wrench::Zero());
            // Copy the arguments to the ID solver
            KDL::JntArray joint_positions((unsigned int)joint_names_.size());
            KDL::JntArray joint_velocities((unsigned int)joint_names_.size());
            KDL::JntArray joint_accelerations((unsigned int)joint_names_.size());
            for (size_t idx = 0; idx < joint_names_.size(); idx++)
            {
                joint_positions((unsigned int)idx) = current_config[idx]; // Current position
                joint_velocities((unsigned int)idx) = target_velocities[idx]; // Desired velocities
                joint_accelerations((unsigned int)idx) = 0.0; // Zero desired accelerations
            }
            // Storage for the computed torques
            KDL::JntArray computed_joint_torques;
            computed_joint_torques.resize((unsigned int)joint_names_.size());
            // Call the ID solver
            int ret = id_solver_->CartToJnt(joint_positions, joint_velocities, joint_accelerations, external_wrench, computed_joint_torques);
            assert(ret >= 0);
            // Copy the results
            std::vector<double> feedforward_torques(joint_names_.size(), 0.0);
            for (size_t idx = 0; idx < joint_names_.size(); idx++)
            {
                feedforward_torques[idx] = computed_joint_torques((unsigned int)idx);
            }
            return feedforward_torques;
        }

        inline std::vector<double> ComputeRawNextStep(const std::vector<double>& current_config, const std::vector<double>& current_velocities, const std::vector<double>& current_accelerations, const std::vector<double>& target_velocities, const double time_interval)
        {
            //const std::chrono::time_point<std::chrono::high_resolution_clock> start = (std::chrono::time_point<std::chrono::high_resolution_clock>)std::chrono::high_resolution_clock::now();
            assert(current_config.size() == joint_names_.size());
            // Compute feedback terms
            const std::vector<double> feedback_terms = ComputeFeedbackTerms(current_velocities, target_velocities, time_interval);
            // Compute feedforward terms
            const std::vector<double> feedforward_terms = ComputeFeedForwardTerms(current_config, current_velocities, current_accelerations, target_velocities);
            // Combine
            const std::vector<double> raw_torque_command = EigenHelpers::Add(feedforward_terms, feedback_terms);
            //const std::chrono::time_point<std::chrono::high_resolution_clock> end = (std::chrono::time_point<std::chrono::high_resolution_clock>)std::chrono::high_resolution_clock::now();
            //const std::chrono::duration<double> elapsed = end - start;
            //const double elapsed_time = elapsed.count();
            //std::cout << "Elapsed FF+FB compute time: " << elapsed_time << std::endl;
            return raw_torque_command;
        }

        inline std::vector<double> EnforceTorqueLimits(const std::vector<double>& raw_torques, const std::vector<std::string>& joint_names, const std::map<std::string, JointLimits>& limits) const
        {
            assert(raw_torques.size() == joint_names.size());
            std::vector<double> limited_torques(raw_torques.size(), 0.0);
            for (size_t idx = 0; idx < limited_torques.size(); idx++)
            {
                const double raw_torque = raw_torques[idx];
                const std::string& joint_name = joint_names[idx];
                const auto found_itr = limits.find(joint_name);
                assert(found_itr != limits.end());
                const double torque_limit = found_itr->second.MaxEffort();
                const double limited_torque = ClampValue(raw_torque, -torque_limit, torque_limit);
                limited_torques[idx] = limited_torque;
            }
            return limited_torques;
        }

        inline std::vector<double> EnforcePositionSafetyLimits(const std::vector<double>& current_positions, const std::vector<double>& raw_torques, const std::vector<std::string>& joint_names, const std::map<std::string, JointLimits>& limits) const
        {
            assert(current_positions.size() == joint_names.size());
            assert(raw_torques.size() == joint_names.size());
            std::vector<double> limited_torques(raw_torques.size(), 0.0);
            for (size_t idx = 0; idx < limited_torques.size(); idx++)
            {
                const double raw_torque = raw_torques[idx];
                const double current_position = current_positions[idx];
                const std::string& joint_name = joint_names[idx];
                const auto found_itr = limits.find(joint_name);
                assert(found_itr != limits.end());
                const std::pair<double, double> limits = found_itr->second.PositionLimits();
                const double lower_limit = limits.first;
                const double upper_limit = limits.second;
                const bool is_under_limits = (current_position < lower_limit) ? true : false;
                const bool is_over_limits = (current_position > upper_limit) ? true : false;
                if (is_under_limits && raw_torque < 0.0)
                {
                    limited_torques[idx] = 0.0;
                }
                else if (is_over_limits && raw_torque > 0.0)
                {
                    limited_torques[idx] = 0.0;
                }
                else
                {
                    limited_torques[idx] = raw_torque;
                }
            }
            return limited_torques;
        }

        inline bool IsSubset(const std::vector<std::string>& candidate_set, const std::vector<std::string>& candidate_subset) const
        {
            if (candidate_set.size() < candidate_subset.size())
            {
                return false;
            }
            std::map<std::string, uint8_t> names_map;
            for (size_t idx = 0; idx < candidate_set.size(); idx++)
            {
                const std::string& name = candidate_set[idx];
                names_map[name] = 0x01;
            }
            for (size_t idx = 0; idx < candidate_subset.size(); idx++)
            {
                const std::string& name = candidate_subset[idx];
                const auto found_itr = names_map.find(name);
                if (found_itr == names_map.end())
                {
                    return false;
                }
            }
            return true;
        }

        inline bool SetsEqual(const std::vector<std::string>& set1, const std::vector<std::string>& set2) const
        {
            if (IsSubset(set1, set2) && IsSubset(set2, set1))
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        inline void VelocityTargetCallback(const iiwa_robot_controllers::VelocityCommandConstPtr& velocity_target)
        {
            if (velocity_target->name.size() == velocity_target->command.size())
            {
                // Push the command into a map
                std::map<std::string, double> command_map;
                for (size_t idx = 0; idx < velocity_target->name.size(); idx++)
                {
                    const std::string& name = velocity_target->name[idx];
                    const double command = velocity_target->command[idx];
                    command_map[name] = command;
                }
                // Extract the joint commands in order
                std::vector<double> target_velocities(joint_names_.size(), 0.0);
                bool command_valid = true;
                for (size_t idx = 0; idx < joint_names_.size(); idx++)
                {
                    // Get the name of the joint
                    const std::string& joint_name = joint_names_[idx];
                    // Get the commanded value
                    const auto found_itr = command_map.find(joint_name);
                    if (found_itr != command_map.end())
                    {
                        const double velocity = found_itr->second;
                        // Get the limit for the joint
                        const auto limit_found_itr = joint_limits_.find(joint_name);
                        // Limit the joint command
                        assert(limit_found_itr != joint_limits_.end());
                        const double limit = limit_found_itr->second.MaxVelocity();
                        const double limited_velocity = ClampValue(velocity, -limit, limit);
                        target_velocities[idx] = limited_velocity;
                    }
                    else
                    {
                        ROS_WARN("Invalid JointCommand: joint %s missing", joint_name.c_str());
                        command_valid = false;
                    }
                }
                if (command_valid == true)
                {
                    target_velocities_lock_.lock();
                    target_velocities_ = target_velocities;
                    target_velocities_valid_.store(true);
                    target_velocities_lock_.unlock();
                }
            }
            else
            {
                ROS_WARN("Invalid JointCommand: %zu names, %zu commands", velocity_target->name.size(), velocity_target->command.size());
                target_velocities_valid_.store(false);
            }
        }
    };
}

