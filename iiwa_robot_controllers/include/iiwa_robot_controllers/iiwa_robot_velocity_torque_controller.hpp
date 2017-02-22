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
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <iiwa_robot_controllers/VelocityCommand.h>
#include <iiwa_robot_controllers/FRICommand.h>
#include <iiwa_robot_controllers/iiwa_robot_config.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

namespace iiwa_robot_controllers
{
    class IIWARobotVelocityTorqueController
    {
    protected:

        std::vector<std::string> joint_names_;
        std::map<std::string, JointLimits> joint_limits_;
        std::map<std::string, PIDParams> joint_controller_params_;

        KDL::Chain chain_;
        std::shared_ptr<KDL::ChainIdSolver_RNE> id_solver_;

        bool target_velocities_valid_;
        bool current_config_valid_;
        std::vector<double> target_velocities_;
        std::vector<double> current_config_;

        uint32_t velocity_filter_window_size_;
        std::vector<std::vector<double>> velocity_filter_window_;

        std::vector<double> current_velocities_;
        std::vector<double> current_accelerations_;
        ros::Time last_config_time_;

        std::vector<double> velocity_error_integrals_;
        std::vector<double> last_velocity_errors_;

        ros::NodeHandle nh_;
        ros::Publisher command_pub_;
        ros::Subscriber feedback_sub_;
        ros::Subscriber velocity_target_sub_;
        ros::ServiceServer abort_server_;

    public:

        IIWARobotVelocityTorqueController(ros::NodeHandle& nh,
                                            const std::string& velocity_command_topic,
                                            const std::string& config_feedback_topic,
                                            const std::string& torque_command_topic,
                                            const std::string& abort_service,
                                            const std::string& xml_model_string,
                                            const bool model_gravity,
                                            const uint32_t velocity_filter_window_size,
                                            const std::map<std::string, JointLimits>& joint_limits,
                                            const std::map<std::string, PIDParams>& joint_controller_params) : nh_(nh)
        {
            target_velocities_valid_ = false;
            current_config_valid_ = false;
            target_velocities_ = std::vector<double>();
            current_config_ = std::vector<double>();
            assert(velocity_filter_window_size >= 1u);
            velocity_filter_window_size_ = velocity_filter_window_size;
            velocity_filter_window_ = std::vector<std::vector<double>>();
            current_velocities_ = std::vector<double>();
            current_accelerations_ = std::vector<double>();
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
            const std::string root_name = "iiwa_link_0";
            const std::string tip_name = "iiwa_link_ee";
            if (!tree.getChain(root_name, tip_name, chain_))
            {
                ROS_ERROR("Could not initialize chain object for root_name %s and tip_name %s", root_name.c_str(), tip_name.c_str());
                assert(false);
            }
            // Setup ID solver
            //const KDL::Vector gravity = model_gravity ? KDL::Vector(0.0, 0.0, -9.81) : KDL::Vector(0.0, 0.0, 0.0);
            const KDL::Vector gravity = model_gravity ? KDL::Vector(0.0, -9.81, 0.0) : KDL::Vector(0.0, 0.0, 0.0);
            id_solver_ = std::shared_ptr<KDL::ChainIdSolver_RNE>(new KDL::ChainIdSolver_RNE(chain_, gravity));
            // Setup publishers and subscribers
            command_pub_ = nh_.advertise<iiwa_robot_controllers::FRICommand>(torque_command_topic, 1, false);
            feedback_sub_ = nh_.subscribe(config_feedback_topic, 1, &IIWARobotVelocityTorqueController::ConfigFeedbackCallback, this);
            velocity_target_sub_ = nh_.subscribe(velocity_command_topic, 1, &IIWARobotVelocityTorqueController::VelocityTargetCallback, this);
            abort_server_ = nh_.advertiseService(abort_service, &IIWARobotVelocityTorqueController::AbortCB, this);
        }

        void Loop(const double control_rate)
        {
            const double control_interval = 1.0 / control_rate;
            ros::Rate spin_rate(control_rate);
            uint8_t iteration_count = 0x00;
            while (ros::ok())
            {
                // Process callbacks
                ros::spinOnce();
                // Run controller
                if (iteration_count == 0x00)
                {
                    if (target_velocities_valid_ && current_config_valid_)
                    {
                        const std::vector<double> raw_torque_command = ComputeRawNextStep(current_config_, current_velocities_, current_accelerations_, target_velocities_, control_interval);
                        //std::cout << "Raw torque command: " << PrettyPrint::PrettyPrint(raw_torque_command) << std::endl;
                        const std::vector<double> limited_torque_command = EnforceTorqueLimits(raw_torque_command, joint_names_, joint_limits_);
                        //std::cout << "Limited torque command: " << PrettyPrint::PrettyPrint(limited_torque_command) << std::endl;
                        const std::vector<double> torque_command = EnforcePositionSafetyLimits(current_config_, limited_torque_command, joint_names_, joint_limits_);
                        //std::cout << "Final torque command: " << PrettyPrint::PrettyPrint(torque_command) << std::endl;
                        CommandTorques(torque_command);
                    }
                    iteration_count = 0x00;
                }
                else
                {
                    iteration_count++;
                }
                // Spin
                spin_rate.sleep();
            }
        }

        inline bool AbortCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
        {
            UNUSED(req);
            UNUSED(res);
            ROS_INFO("Aborting config target");
            target_velocities_valid_ = false;
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

        inline void CommandTorques(const std::vector<double>& torques)
        {
            if (torques.size() == joint_names_.size())
            {
                iiwa_robot_controllers::FRICommand command_msg;
                command_msg.mode = iiwa_robot_controllers::FRICommand::TORQUE;
                command_msg.joint_name = joint_names_;
                command_msg.joint_command = torques;
                command_pub_.publish(command_msg);
            }
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

        inline void VelocityTargetCallback(iiwa_robot_controllers::VelocityCommand velocity_target)
        {
            if (velocity_target.name.size() == velocity_target.command.size())
            {
                // Push the command into a map
                std::map<std::string, double> command_map;
                for (size_t idx = 0; idx < velocity_target.name.size(); idx++)
                {
                    const std::string& name = velocity_target.name[idx];
                    const double command = velocity_target.command[idx];
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
                    target_velocities_ = target_velocities;
                    target_velocities_valid_ = true;
                }
            }
            else
            {
                ROS_WARN("Invalid JointCommand: %zu names, %zu commands", velocity_target.name.size(), velocity_target.command.size());
                target_velocities_valid_ = false;
            }
        }

        inline std::vector<double> FilterVelocities(const std::vector<std::vector<double>>& velocities) const
        {
            std::vector<Eigen::VectorXd> window_velocities(velocities.size());
            for (size_t idx = 0; idx < velocities.size(); idx++)
            {
                window_velocities[idx] = EigenHelpers::StdVectorDoubleToEigenVectorXd(velocities[idx]);
            }
            const Eigen::VectorXd average_velocity = EigenHelpers::AverageEigenVectorXd(window_velocities);
            return EigenHelpers::EigenVectorXdToStdVectorDouble(average_velocity);
        }

        inline void ConfigFeedbackCallback(sensor_msgs::JointState config_feedback)
        {
            if ((config_feedback.name.size() == config_feedback.position.size()) && (config_feedback.name.size() == config_feedback.velocity.size()) && IsSubset(config_feedback.name, joint_names_))
            {
                // Push the joint state into a map
                std::map<std::string, std::pair<double, double>> joint_state_map;
                for (size_t idx = 0; idx < config_feedback.name.size(); idx++)
                {
                    const std::string& name = config_feedback.name[idx];
                    const double position = config_feedback.position[idx];
                    const double velocity = config_feedback.velocity[idx];
                    joint_state_map[name] = std::make_pair(position, velocity);
                }
                // Extract the joint state in order
                std::vector<double> current_config(joint_names_.size(), 0.0);
                std::vector<double> current_velocities(joint_names_.size(), 0.0);
                bool config_valid = true;
                for (size_t idx = 0; idx < joint_names_.size(); idx++)
                {
                    const std::string& joint_name = joint_names_[idx];
                    const auto found_itr = joint_state_map.find(joint_name);
                    if (found_itr != joint_state_map.end())
                    {
                        const double position = found_itr->second.first;
                        const double velocity = found_itr->second.second;
                        current_config[idx] = position;
                        current_velocities[idx] = velocity;
                    }
                    else
                    {
                        ROS_WARN("Invalid JointState feedback: joint %s missing", joint_name.c_str());
                        config_valid = false;
                    }
                }
                if (config_valid == true)
                {
                    // Update the velocity filter window
                    if (velocity_filter_window_.size() < velocity_filter_window_size_)
                    {
                        velocity_filter_window_.push_back(current_velocities);
                    }
                    else
                    {
                        velocity_filter_window_.erase(velocity_filter_window_.begin(), velocity_filter_window_.begin() + 1);
                        velocity_filter_window_.push_back(current_velocities);
                    }
                    // If our filter window is full, keep going
                    if (velocity_filter_window_.size() == velocity_filter_window_size_)
                    {
                        current_config_ = current_config;
                        const std::vector<double> last_velocities = current_velocities_;
                        current_velocities_ = FilterVelocities(velocity_filter_window_);
                        // Compute accelerations if we have valid previous data
                        if (current_config_valid_)
                        {
                            const ros::Time& current_time = config_feedback.header.stamp;
                            const ros::Time& previous_time = last_config_time_;
                            const ros::Duration interval = current_time - previous_time;
                            const std::vector<double> velocity_delta = EigenHelpers::Sub(current_velocities_, last_velocities);
                            const double time_interval = interval.toSec();
                            const std::vector<double> accelerations = EigenHelpers::Divide(velocity_delta, time_interval);
                            current_accelerations_ = accelerations;
                        }
                        else
                        {
                            current_accelerations_ = std::vector<double>(current_velocities.size(), 0.0);
                        }
                        current_config_valid_ = true;
                        last_config_time_ = config_feedback.header.stamp;
                    }
                    else
                    {
                        ROS_INFO("Waiting to fill velocity filter window for the first time...");
                        current_config_valid_ = false;
                    }
                }
            }
            else
            {
                ROS_WARN("Invalid JointState feedback: %zu names, %zu positions, %zu velocities", config_feedback.name.size(), config_feedback.position.size(), config_feedback.velocity.size());
                current_config_valid_ = false;
            }
        }
    };
}

