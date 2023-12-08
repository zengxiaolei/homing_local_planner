/*********************************************************************
 *
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2023
 *
 * Author: Lei ZENG
 *********************************************************************/

#ifndef HOMING_LOCAL_PLANNER_HPP_
#define HOMING_LOCAL_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "Eigen/Core"
#include "eigen3/Eigen/Dense"

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "homing_local_planner/visualization.hpp"

namespace homing_local_planner
{
    class HomingLocalPlanner : public nav2_core::Controller
    {
    public:
        HomingLocalPlanner() = default;

        ~HomingLocalPlanner() override = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::SharedPtr &parent,
            std::string name, const std::shared_ptr<tf2_ros::Buffer> &tf,
            const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros) override;

        void cleanup() override;

        void activate() override;

        void deactivate() override;

        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped &pose,
            const geometry_msgs::msg::Twist &velocity) override;

        void setPlan(const nav_msgs::msg::Path &path) override;

    protected:
        template <typename P1, typename P2>

        inline double distancePoints2d(const P1 &point1, const P2 &point2)
        {
            return std::sqrt(std::pow(point2.x - point1.x, 2) + std::pow(point2.y - point1.y, 2));
        }

        void simplifyGlobalPlan(std::vector<geometry_msgs::msg::PoseStamped> &plan, double simplify_separation);

        bool pruneGlobalPlan(const std::shared_ptr<tf2_ros::Buffer> &tf, const geometry_msgs::msg::PoseStamped &global_pose,
                             std::vector<geometry_msgs::msg::PoseStamped> &global_plan, double dist_behind_robot = 1);

        void smoothPlan2d(std::vector<geometry_msgs::msg::PoseStamped> &global_plan_vec);

        void updateViaPointsContainer(const std::vector<geometry_msgs::msg::PoseStamped> &transformed_plan,
                                      double min_separation_via, double min_separation_goal);

        bool transformGlobalPlan(const std::shared_ptr<tf2_ros::Buffer> &tf,
                                 const std::vector<geometry_msgs::msg::PoseStamped> &global_plan,
                                 const geometry_msgs::msg::PoseStamped &global_pose,
                                 const nav2_costmap_2d::Costmap2D &costmap,
                                 const std::string &global_frame, double max_plan_length,
                                 std::vector<geometry_msgs::msg::PoseStamped> &transformed_plan);

        double clip(double value, double lower, double upper);

        void cart2Pol(double x, double y, double &deg, double &dist);

        double angDiff(double alpha, double beta);

        void poseError(double dx, double dy, double dyaw, double &rho, double &alpha, double &phi);

        void homingControl(double rho, double alpha, double phi, double &v, double &omega);

        void homingControl2(double dx, double dy, double yaw, double yaw_goal, double &v, double &omega);

        Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R);

    private:
        std::string plugin_name_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        nav2_costmap_2d::Costmap2D *costmap_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        rclcpp::Logger logger_{rclcpp::get_logger("HomingLocalPlanner")};

        bool initialized_;
        double xy_reached_ = false;
        bool last_back_ = false;
        // double control_duration_;

        std::string global_frame_;     //!< The frame in which the controller will run
        std::string robot_base_frame_; //!< Used as the base frame id of the robot
        geometry_msgs::msg::Twist last_cmd_;
        geometry_msgs::msg::PoseStamped robot_pose_;

        std::vector<geometry_msgs::msg::Point> transformed_footprint_;
        std::vector<geometry_msgs::msg::PoseStamped> global_plan_vec_;
        std::vector<geometry_msgs::msg::PoseStamped> local_plan_vec_;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> via_points_vec_;

        nav_msgs::msg::Path global_plan_, local_plan_;
        HomingVisualizationPtr visualization_;

        double optimization_k_alpha_;
        double optimization_k_rho_;
        double optimization_k_phi_;

        double robot_max_vel_x_;
        double robot_max_vel_theta_;
        bool robot_turn_around_priority_;
        // double robot_max_acc_theta_;
        // double robot_min_turning_raduis_;

        double trajectory_global_plan_prune_distance_;
        double trajectory_max_global_plan_lookahead_dist_;
        double trajectory_global_plan_viapoint_sep_;
        double trajectory_global_plan_goal_sep_;

        double goal_tolerance_xy_goal_tolerance_;
        double goal_tolerance_yaw_goal_tolerance_;
    };
}

#endif