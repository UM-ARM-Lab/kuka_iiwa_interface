#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <random>
#include <Eigen/Geometry>
#include <arc_utilities/arc_helpers.hpp>

#ifndef IIWA_ROBOT_CONFIG_HPP
#define IIWA_ROBOT_CONFIG_HPP

namespace iiwa_robot_controllers
{
    class PIDParams
    {
    protected:

        double kp_;
        double ki_;
        double kd_;
        double i_clamp_;

    public:

        PIDParams(const double kp, const double ki, const double kd, const double i_clamp) : kp_(kp), ki_(ki), kd_(kd), i_clamp_(i_clamp) {}

        PIDParams() : kp_(0.0), ki_(0.0), kd_(0.0), i_clamp_(0.0) {}

        inline double Kp() const
        {
            return kp_;
        }

        inline double Ki() const
        {
            return ki_;
        }

        inline double Kd() const
        {
            return kd_;
        }

        inline double Iclamp() const
        {
            return i_clamp_;
        }
    };

    class JointLimits
    {
    protected:

        double min_position_;
        double max_position_;
        double max_velocity_;
        double max_effort_;

    public:

        JointLimits(const double min_position, const double max_position, const double max_velocity, const double max_effort)
        {
            assert(min_position <= max_position);
            min_position_ = min_position;
            max_position_ = max_position;
            max_velocity_ = std::abs(max_velocity);
            max_effort_ = std::abs(max_effort);
        }

        JointLimits() : min_position_(0.0), max_position_(0.0), max_velocity_(0.0), max_effort_(0.0) {}

        inline double MinPosition() const
        {
            return min_position_;
        }

        inline double MaxPosition() const
        {
            return max_position_;
        }

        inline std::pair<double, double> PositionLimits() const
        {
            return std::make_pair(min_position_, max_position_);
        }

        inline double MaxVelocity() const
        {
            return max_velocity_;
        }

        inline double MaxEffort() const
        {
            return max_effort_;
        }
    };

    inline std::vector<std::string> GetJointNames(const std::string& joint_name_prefix)
    {
        std::vector<std::string> joint_names(7);
        joint_names[0] = joint_name_prefix + "_joint_1";
        joint_names[1] = joint_name_prefix + "_joint_2";
        joint_names[2] = joint_name_prefix + "_joint_3";
        joint_names[3] = joint_name_prefix + "_joint_4";
        joint_names[4] = joint_name_prefix + "_joint_5";
        joint_names[5] = joint_name_prefix + "_joint_6";
        joint_names[6] = joint_name_prefix + "_joint_7";
        return joint_names;
    }

    inline std::map<std::string, JointLimits> GetArmDefaultLimits(const std::string& joint_name_prefix)
    {
        const std::vector<std::string> joint_names = GetJointNames(joint_name_prefix);
        std::map<std::string, JointLimits> joint_limits;
        joint_limits[joint_names[0]] = JointLimits(-2.96705972839, 2.96705972839, 10.0, 300.0);
        joint_limits[joint_names[1]] = JointLimits(-2.09439510239, 2.09439510239, 10.0, 300.0);
        joint_limits[joint_names[2]] = JointLimits(-2.96705972839, 2.96705972839, 10.0, 300.0);
        joint_limits[joint_names[3]] = JointLimits(-2.09439510239, 2.09439510239, 10.0, 300.0);
        joint_limits[joint_names[4]] = JointLimits(-2.96705972839, 2.96705972839, 10.0, 300.0);
        joint_limits[joint_names[5]] = JointLimits(-2.09439510239, 2.09439510239, 10.0, 300.0);
        joint_limits[joint_names[6]] = JointLimits(-3.05432619099, 3.05432619099, 10.0, 300.0);
        return joint_limits;
    }

    inline std::map<std::string, JointLimits> GetArmLimits(const std::string& joint_name_prefix, const double position_scaling=1.0, const double velocity_scaling=1.0, const double effort_scaling=1.0)
    {
        assert(position_scaling >= 0.0);
        assert(position_scaling <= 1.0);
        assert(velocity_scaling >= 0.0);
        assert(velocity_scaling <= 1.0);
        assert(effort_scaling >= 0.0);
        assert(effort_scaling <= 1.0);
        const std::map<std::string, JointLimits> default_joint_limits = GetArmDefaultLimits(joint_name_prefix);
        std::map<std::string, JointLimits> joint_limits;
        const auto default_limit_pairs = arc_helpers::GetKeysAndValues(default_joint_limits);
        for (size_t idx = 0; idx < default_limit_pairs.size(); idx++)
        {
            const std::string& joint_name = default_limit_pairs[idx].first;
            const JointLimits& default_joint_limit = default_limit_pairs[idx].second;
            const JointLimits scaled_joint_limit((default_joint_limit.MinPosition() * position_scaling),
                                                 (default_joint_limit.MaxPosition() * position_scaling),
                                                 (default_joint_limit.MaxVelocity() * velocity_scaling),
                                                 (default_joint_limit.MaxEffort() * effort_scaling));
            joint_limits[joint_name] = scaled_joint_limit;
        }
        return joint_limits;
    }

    inline std::map<std::string, PIDParams> GetArmDefaultPositionControllerParams(const std::string& joint_name_prefix)
    {
        const std::vector<std::string> joint_names = GetJointNames(joint_name_prefix);
        std::map<std::string, iiwa_robot_controllers::PIDParams> joint_controller_params;
        joint_controller_params[joint_names[0]] = iiwa_robot_controllers::PIDParams(1.0, 0.0, 0.1, 1.0);
        joint_controller_params[joint_names[1]] = iiwa_robot_controllers::PIDParams(1.0, 0.0, 0.1, 1.0);
        joint_controller_params[joint_names[2]] = iiwa_robot_controllers::PIDParams(1.0, 0.0, 0.1, 1.0);
        joint_controller_params[joint_names[3]] = iiwa_robot_controllers::PIDParams(1.0, 0.0, 0.1, 1.0);
        joint_controller_params[joint_names[4]] = iiwa_robot_controllers::PIDParams(1.0, 0.0, 0.1, 1.0);
        joint_controller_params[joint_names[5]] = iiwa_robot_controllers::PIDParams(1.0, 0.0, 0.1, 1.0);
        joint_controller_params[joint_names[6]] = iiwa_robot_controllers::PIDParams(1.0, 0.0, 0.1, 1.0);
        return joint_controller_params;
    }

    inline std::map<std::string, PIDParams> GetArmDefaultVelocityControllerParams(const std::string& joint_name_prefix)
    {
        const std::vector<std::string> joint_names = GetJointNames(joint_name_prefix);
        std::map<std::string, iiwa_robot_controllers::PIDParams> joint_controller_params;
        joint_controller_params[joint_names[0]] = iiwa_robot_controllers::PIDParams(10.0, 0.0, 1.0, 0.0);
        joint_controller_params[joint_names[1]] = iiwa_robot_controllers::PIDParams(10.0, 0.0, 1.0, 0.0);
        joint_controller_params[joint_names[2]] = iiwa_robot_controllers::PIDParams(10.0, 0.0, 1.0, 0.0);
        joint_controller_params[joint_names[3]] = iiwa_robot_controllers::PIDParams(10.0, 0.0, 1.0, 0.0);
        joint_controller_params[joint_names[4]] = iiwa_robot_controllers::PIDParams(10.0, 0.0, 1.0, 0.0);
        joint_controller_params[joint_names[5]] = iiwa_robot_controllers::PIDParams(10.0, 0.0, 1.0, 0.0);
        joint_controller_params[joint_names[6]] = iiwa_robot_controllers::PIDParams(10.0, 0.0, 1.0, 0.0);
        return joint_controller_params;
    }

}
#endif // IIWA_ROBOT_CONFIG_HPP
