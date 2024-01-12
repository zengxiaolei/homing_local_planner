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

#include <homing_local_planner/homing_config.h>
namespace homing_local_planner
{
    void HomingConfig::loadRosParamFromNodeHandle(const ros::NodeHandle &nh)
    {
        // Robot
        nh.param("max_vel_x", robot.max_vel_x, robot.max_vel_x);
        nh.param("max_vel_x_backwards", robot.max_vel_x_backwards, robot.max_vel_x_backwards);
        nh.param("max_vel_theta", robot.max_vel_theta, robot.max_vel_theta);
        nh.param("acc_lim_x", robot.acc_lim_x, robot.acc_lim_x);
        nh.param("acc_lim_theta", robot.acc_lim_theta, robot.acc_lim_theta);
        nh.param("min_turn_radius", robot.min_turn_radius, robot.min_turn_radius);
        nh.param("turn_around_priority", robot.turn_around_priority, robot.turn_around_priority);
        nh.param("stop_dist", robot.stop_dist, robot.stop_dist);
        nh.param("dec_dist", robot.dec_dist, robot.dec_dist);

        // GoalTolerance
        nh.param("xy_goal_tolerance", goal_tolerance.xy_goal_tolerance, goal_tolerance.xy_goal_tolerance);
        nh.param("yaw_goal_tolerance", goal_tolerance.yaw_goal_tolerance, goal_tolerance.yaw_goal_tolerance);

        // Trajectory
        nh.param("max_global_plan_lookahead_dist", trajectory.max_global_plan_lookahead_dist, trajectory.max_global_plan_lookahead_dist);
        nh.param("global_plan_viapoint_sep", trajectory.global_plan_viapoint_sep, trajectory.global_plan_viapoint_sep);
        nh.param("global_plan_goal_sep", trajectory.global_plan_goal_sep, trajectory.global_plan_goal_sep);
        nh.param("global_plan_prune_distance", trajectory.global_plan_prune_distance, trajectory.global_plan_prune_distance);

        // Optimization
        nh.param("k_rho", optimization.k_rho, optimization.k_rho);
        nh.param("k_alpha", optimization.k_alpha, optimization.k_alpha);
        nh.param("k_phi", optimization.k_phi, optimization.k_phi);
    }

    void HomingConfig::reconfigure(HomingLocalPlannerReconfigureConfig &cfg)
    {
        boost::mutex::scoped_lock l(config_mutex_);
        // Robot
        robot.max_vel_x = cfg.max_vel_x;
        robot.max_vel_x_backwards = cfg.max_vel_x_backwards;
        robot.max_vel_theta = cfg.max_vel_theta;
        robot.acc_lim_x = cfg.acc_lim_x;
        robot.acc_lim_theta = cfg.acc_lim_theta;
        robot.min_turn_radius = cfg.min_turn_radius;
        robot.turn_around_priority = cfg.turn_around_priority;
        robot.stop_dist = cfg.stop_dist;
        robot.dec_dist = cfg.dec_dist;

        // GoalTolerance
        goal_tolerance.xy_goal_tolerance = cfg.xy_goal_tolerance;
        goal_tolerance.yaw_goal_tolerance = cfg.yaw_goal_tolerance;

        // Trajectory
        trajectory.max_global_plan_lookahead_dist = cfg.max_global_plan_lookahead_dist;
        trajectory.global_plan_viapoint_sep = cfg.global_plan_viapoint_sep;
        trajectory.global_plan_goal_sep = cfg.global_plan_goal_sep;
        trajectory.global_plan_prune_distance = cfg.global_plan_prune_distance;

        // Optimization
        optimization.k_rho = cfg.k_rho;
        optimization.k_alpha = cfg.k_alpha;
        optimization.k_phi = cfg.k_phi;
    }
}