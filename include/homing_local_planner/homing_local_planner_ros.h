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

#ifndef HOMING_LOCAL_PLANNER_ROS_H_
#define HOMING_LOCAL_PLANNER_ROS_H_

#include <ros/ros.h>

#include <nav_core/base_local_planner.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/point_grid.h>
#include <base_local_planner/costmap_model.h>

#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/footprint.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include "Eigen/Core"
#include "eigen3/Eigen/Dense"
#include <cmath>
#include <algorithm>

#include <homing_local_planner/visualization.h>
#include <homing_local_planner/homing_config.h>
#include <homing_local_planner/one_euro_filter.h>
#include <homing_local_planner/obstacles.h>
#include <homing_local_planner/pose_se2.h>
#include <homing_local_planner/robot_footprint_model.h>

namespace homing_local_planner
{
    class HomingLocalPlanner : public nav_core::BaseLocalPlanner
    {
    public:
        HomingLocalPlanner();

        ~HomingLocalPlanner();

        void initialize(std::string name, tf2_ros::Buffer *tf,
                        costmap_2d::Costmap2DROS *costmap_ros);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

        bool isGoalReached();

        static RobotFootprintModelPtr getRobotFootprintFromParamServer(const ros::NodeHandle &nh);

        static Point2dContainer makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue &footprint_xmlrpc, const std::string &full_param_name);

        static double getNumberFromXMLRPC(XmlRpc::XmlRpcValue &value, const std::string &full_param_name);

    protected:
        template <typename P1, typename P2>

        inline double distancePoints2d(const P1 &point1, const P2 &point2)
        {
            return std::sqrt(std::pow(point2.x - point1.x, 2) + std::pow(point2.y - point1.y, 2));
        };

        bool pruneGlobalPlan(const tf2_ros::Buffer &tf, const geometry_msgs::PoseStamped &global_pose,
                             std::vector<geometry_msgs::PoseStamped> &global_plan, double dist_behind_robot = 1);

        void smoothPlan2d(std::vector<geometry_msgs::PoseStamped> &global_plan);

        void simplifyGlobalPlan(std::vector<geometry_msgs::PoseStamped> &plan, double simplify_separation);

        void updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped> &transformed_plan,
                                      double min_separation_via, double min_separation_goal);

        void reconfigureCB(HomingLocalPlannerReconfigureConfig &config, uint32_t level);

        double clip(double value, double lower, double upper);

        void cart2Pol(double x, double y, double &deg, double &dist);

        double angDiff(double alpha, double beta);

        void poseError(double dx, double dy, double dyaw, double &rho, double &alpha, double &phi);

        void homingControl(double rho, double alpha, double phi, double &v, double &omega);

        void homingControl2(double dx, double dy, double yaw, double yaw_goal, double &v, double &omega);

        Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R);

        void updateObstacleContainerWithCostmap();

        bool checkCollision(const std::vector<geometry_msgs::PoseStamped> &transformed_plan, double obst_dist);

        bool transformGlobalPlan(const tf2_ros::Buffer &tf, const std::vector<geometry_msgs::PoseStamped> &global_plan,
                                 const geometry_msgs::PoseStamped &global_pose, const costmap_2d::Costmap2D &costmap,
                                 const std::string &global_frame, double max_plan_length, std::vector<geometry_msgs::PoseStamped> &transformed_plan,
                                 int *current_goal_idx = NULL, geometry_msgs::TransformStamped *tf_plan_to_global = NULL) const;

    private:
        costmap_2d::Costmap2DROS *costmap_ros_;
        costmap_2d::Costmap2D *costmap_;
        tf2_ros::Buffer *tf_;

        bool initialized_;
        bool goal_reached_;
        double xy_reached_ = false;
        bool last_back_ = false;

        std::string global_frame_;     //!< The frame in which the controller will run
        std::string robot_base_frame_; //!< Used as the base frame id of the robot
        geometry_msgs::Twist last_cmd_;
        geometry_msgs::PoseStamped robot_pose_; //!< Store current robot pose                                  //!< store whether the goal is reached or not
        PoseSE2 robot_pose_se2_;
        std::vector<geometry_msgs::Point> transformed_footprint_;
        std::vector<geometry_msgs::PoseStamped> global_plan_; //!< Store the current global plan
        std::vector<geometry_msgs::PoseStamped> local_plan_;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> via_points_;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> collision_points_;

        HomingVisualizationPtr visualization_;
        HomingConfig cfg_;
        ObstContainer obstacles_;
        RobotFootprintModelPtr robot_model_;
        base_local_planner::WorldModel *world_model_;
        boost::shared_ptr<dynamic_reconfigure::Server<HomingLocalPlannerReconfigureConfig>> dynamic_recfg_; //!< Dynamic reconfigure server to allow config modifications at runtime
        OneEuroFilter omega_filter_ = OneEuroFilter(ros::Time::now(), 0.0, 0.0, 1.0, 0.3, 1.0);
    };
};

#endif
