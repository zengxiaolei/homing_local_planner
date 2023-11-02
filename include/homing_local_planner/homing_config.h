/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Lei ZENG
 *********************************************************************/

#ifndef HOMING_CONFIG_H_
#define HOMING_CONFIG_H_

#include <ros/console.h>
#include <ros/ros.h>
#include <homing_local_planner/HomingLocalPlannerReconfigureConfig.h>

namespace homing_local_planner
{
    class HomingConfig
    {
    public:
        //! Robot related parameters
        struct Robot
        {
            double max_vel_x;           //!< Maximum translational velocity of the robot
            double max_vel_x_backwards; //!< Maximum translational velocity of the robot for driving backwards
            double max_vel_theta;       //!< Maximum angular velocity of the robot
            double acc_lim_x;           //!< Maximum translational acceleration of the robot
            double acc_lim_theta;       //!< Maximum angular acceleration of the robot
        } robot;

        //! Goal tolerance related parameters
        struct GoalTolerance
        {
            double yaw_goal_tolerance; //!< Allowed final orientation error
            double xy_goal_tolerance;  //!< Allowed final euclidean distance to the goal position
        } goal_tolerance;              //!< Goal tolerance related parameters

        //! Trajectory related parameters
        struct Trajectory
        {
            double max_global_plan_lookahead_dist; //!< Specify maximum length (cumulative Euclidean distances) of the subset of the global plan taken into account for optimization [if <=0: disabled; the length is also bounded by the local costmap size!]
            double global_plan_viapoint_sep;       //!< Min. separation between each two consecutive via-points extracted from the global plan (if negative: disabled)
            double global_plan_goal_sep;
            double global_plan_prune_distance; //!< Distance between robot and via_points of global plan which is used for pruning
        } trajectory;

        //! Optimization related parameters
        struct Optimization
        {
            double k_rho;
            double k_alpha;
            double k_phi;
        } optimization;

        HomingConfig()
        {
            // Robot
            robot.max_vel_x = 0.3;
            robot.max_vel_x_backwards = 0.2;
            robot.max_vel_theta = 0.5;
            robot.acc_lim_x = 0.2;
            robot.acc_lim_theta = 0.2;

            // GoalTolerance
            goal_tolerance.xy_goal_tolerance = 0.2;
            goal_tolerance.yaw_goal_tolerance = 0.2;

            // Trajectory
            trajectory.max_global_plan_lookahead_dist = 3.0;
            trajectory.global_plan_viapoint_sep = 0.5;
            trajectory.global_plan_goal_sep = 0.8;
            trajectory.global_plan_prune_distance = 0.2;

            // Optimization
            optimization.k_rho = 1.0;
            optimization.k_alpha = -3.0;
            optimization.k_phi = -1.0;
        }
        void loadRosParamFromNodeHandle(const ros::NodeHandle &nh);
        void reconfigure(HomingLocalPlannerReconfigureConfig &cfg);

        /**
         * @brief Return the internal config mutex
         */
        boost::mutex &configMutex() { return config_mutex_; }

    private:
        boost::mutex config_mutex_; //!< Mutex for config accesses and changes
    };
};
#endif