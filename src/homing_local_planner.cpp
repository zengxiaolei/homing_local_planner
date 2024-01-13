/*********************************************************************
 *
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2023
 *
 * Author: Lei ZENG
 *********************************************************************/

#include <algorithm>
#include <string>
#include <memory>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"

#include "homing_local_planner/homing_local_planner.hpp"
#include "homing_local_planner/misc.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace homing_local_planner
{

    void HomingLocalPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        auto node = parent.lock();
        node_ = parent;
        if (!node)
        {
            throw nav2_core::PlannerException("Unable to lock node!");
        }

        tf_ = tf;
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();

        plugin_name_ = name;
        logger_ = node->get_logger();
        global_frame_ = costmap_ros_->getGlobalFrameID();
        robot_base_frame_ = costmap_ros_->getBaseFrameID();

        visualization_ = HomingVisualizationPtr(new HomingVisualization(node, global_frame_));

        // double control_frequency = 20.0;

        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".optimization_k_alpha", rclcpp::ParameterValue(-3.0));
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".optimization_k_rho", rclcpp::ParameterValue(1.0));
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".optimization_k_phi", rclcpp::ParameterValue(-1.0));

        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".robot_max_vel_x", rclcpp::ParameterValue(0.2));
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".robot_max_vel_theta", rclcpp::ParameterValue(0.4));
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".robot_min_turn_radius", rclcpp::ParameterValue(0.0));
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".robot_turn_around_priority", rclcpp::ParameterValue(true));
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".robot_stop_dist", rclcpp::ParameterValue(0.5));
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".robot_dec_dist", rclcpp::ParameterValue(1.0));

        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".trajectory_global_plan_prune_distance", rclcpp::ParameterValue(1.0));
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".trajectory_max_global_plan_lookahead_dist", rclcpp::ParameterValue(3.0));
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".trajectory_global_plan_viapoint_sep", rclcpp::ParameterValue(0.3));
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".trajectory_global_plan_goal_sep", rclcpp::ParameterValue(0.5));

        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".goal_tolerance_xy_goal_tolerance", rclcpp::ParameterValue(0.2));
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".goal_tolerance_yaw_goal_tolerance", rclcpp::ParameterValue(0.2));

        node->get_parameter(plugin_name_ + ".optimization_k_alpha", optimization_k_alpha_);
        node->get_parameter(plugin_name_ + ".optimization_k_rho", optimization_k_rho_);
        node->get_parameter(plugin_name_ + ".optimization_k_phi", optimization_k_phi_);

        node->get_parameter(plugin_name_ + ".robot_max_vel_x", robot_max_vel_x_);
        node->get_parameter(plugin_name_ + ".robot_max_vel_theta", robot_max_vel_theta_);
        node->get_parameter(plugin_name_ + ".robot_min_turn_radius", robot_min_turn_radius_);
        node->get_parameter(plugin_name_ + ".robot_turn_around_priority", robot_turn_around_priority_);
        node->get_parameter(plugin_name_ + ".robot_stop_dist", robot_stop_dist_);
        node->get_parameter(plugin_name_ + ".robot_dec_dist", robot_dec_dist_);

        node->get_parameter(plugin_name_ + ".trajectory_global_plan_prune_distance", trajectory_global_plan_prune_distance_);
        node->get_parameter(plugin_name_ + ".trajectory_max_global_plan_lookahead_dist", trajectory_max_global_plan_lookahead_dist_);
        node->get_parameter(plugin_name_ + ".trajectory_global_plan_viapoint_sep", trajectory_global_plan_viapoint_sep_);
        node->get_parameter(plugin_name_ + ".trajectory_global_plan_goal_sep", trajectory_global_plan_goal_sep_);

        node->get_parameter(plugin_name_ + ".goal_tolerance_xy_goal_tolerance", goal_tolerance_xy_goal_tolerance_);
        node->get_parameter(plugin_name_ + ".goal_tolerance_yaw_goal_tolerance", goal_tolerance_yaw_goal_tolerance_);

        // node->get_parameter("controller_frequency", control_frequency);
        // control_duration_ = 1.0 / control_frequency;
    }

    void HomingLocalPlanner::cleanup()
    {
        visualization_->cleanup();
    }

    void HomingLocalPlanner::activate()
    {
        visualization_->activate();
        RCLCPP_INFO(
            logger_,
            "Activating controller:homing local planner");
        auto node = node_.lock();
        dyn_params_handler_ = node->add_on_set_parameters_callback(
            std::bind(
                &HomingLocalPlanner::dynamicParametersCallback,
                this, std::placeholders::_1));
    }

    void HomingLocalPlanner::deactivate()
    {
        visualization_->deactivate();
        dyn_params_handler_.reset();
    }

    void HomingLocalPlanner::setPlan(const nav_msgs::msg::Path &path)
    {
        global_plan_ = path;

        global_plan_vec_.clear();
        global_plan_vec_ = std::vector<geometry_msgs::msg::PoseStamped>(std::begin(global_plan_.poses), std::end(global_plan_.poses));

        xy_reached_ = false;
        last_back_ = false;

        simplifyGlobalPlan(global_plan_vec_, 0.03);
        smoothPlan2d(global_plan_vec_);
        global_plan_.poses = global_plan_vec_;
    }

    void HomingLocalPlanner::setSpeedLimit(
        const double &speed_limit,
        const bool &percentage)
    {
        if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT)
        {
        }
        else
        {
            if (percentage)
            {
                robot_max_vel_x_ = robot_max_vel_x_ * speed_limit / 100.0;
            }
            else
            {
                robot_max_vel_x_ = speed_limit;
            }
        }
    }

    rcl_interfaces::msg::SetParametersResult
    HomingLocalPlanner::dynamicParametersCallback(
        std::vector<rclcpp::Parameter> parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        std::lock_guard<std::mutex> lock_reinit(mutex_);
        for (auto parameter : parameters)
        {
            const auto &type = parameter.get_type();
            const auto &name = parameter.get_name();
            if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
            {
                if (name == plugin_name_ + ".optimization_k_alpha")
                {
                    optimization_k_alpha_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".optimization_k_rho")
                {
                    optimization_k_rho_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".optimization_k_phi")
                {
                    optimization_k_phi_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".robot_max_vel_x")
                {
                    robot_max_vel_x_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".robot_max_vel_theta")
                {
                    robot_max_vel_theta_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".robot_min_turn_radius")
                {
                    robot_min_turn_radius_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".robot_stop_dist")
                {
                    robot_stop_dist_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".robot_dec_dist")
                {
                    robot_dec_dist_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".trajectory_global_plan_prune_distance")
                {
                    trajectory_global_plan_prune_distance_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".trajectory_max_global_plan_lookahead_dist")
                {
                    trajectory_max_global_plan_lookahead_dist_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".trajectory_global_plan_viapoint_sep")
                {
                    trajectory_global_plan_viapoint_sep_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".trajectory_global_plan_goal_sep")
                {
                    trajectory_global_plan_goal_sep_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".goal_tolerance_xy_goal_tolerance")
                {
                    goal_tolerance_xy_goal_tolerance_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".goal_tolerance_yaw_goal_tolerance")
                {
                    goal_tolerance_yaw_goal_tolerance_ = parameter.as_double();
                }
            }
            else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
            {
                if (name == plugin_name_ + ".robot_turn_around_priority")
                {
                    robot_turn_around_priority_ = parameter.as_bool();
                }
            }
        }

        result.successful = true;
        return result;
    }

    geometry_msgs::msg::TwistStamped HomingLocalPlanner::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &speed,
        nav2_core::GoalChecker *goal_checker)
    {
        std::lock_guard<std::mutex> lock_reinit(mutex_);

        // Update for the current goal checker's state
        geometry_msgs::msg::Pose pose_tolerance;
        geometry_msgs::msg::Twist vel_tolerance;
        if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance))
        {
            RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
        }

        geometry_msgs::msg::TwistStamped cmd_vel;
        robot_pose_ = pose;
        pruneGlobalPlan(tf_, robot_pose_, global_plan_vec_, trajectory_global_plan_prune_distance_);

        RCLCPP_DEBUG(logger_, "speed linear:%f, angular: %f", speed.linear.x, speed.angular.z); // todo

        std::vector<geometry_msgs::msg::PoseStamped> transformed_plan; // in global_frame_
        if (!transformGlobalPlan(tf_, global_plan_vec_, robot_pose_, *costmap_,
                                 global_frame_, trajectory_max_global_plan_lookahead_dist_,
                                 transformed_plan))
        {
            RCLCPP_WARN(logger_, "Could not transform the global plan to the frame of the controller ");
            return cmd_vel;
        }

        if (transformed_plan.empty())
        {

            RCLCPP_WARN(logger_, "transformed plan empty");
            return cmd_vel;
        }

        const geometry_msgs::msg::PoseStamped &goal_point = transformed_plan.back();

        // we assume the global goal is the last point in the global plan
        const double goal_x = goal_point.pose.position.x;
        const double goal_y = goal_point.pose.position.y;
        const double goal_th = tf2::getYaw(goal_point.pose.orientation);
        double dx = goal_x - robot_pose_.pose.position.x;
        double dy = goal_y - robot_pose_.pose.position.y;
        double dyaw = fmod(goal_th - tf2::getYaw(robot_pose_.pose.orientation), 2 * M_PI);

        if ((fabs(std::sqrt(dx * dx + dy * dy)) < goal_tolerance_xy_goal_tolerance_) && (fabs(dyaw) >= goal_tolerance_yaw_goal_tolerance_))
            xy_reached_ = true;

        updateViaPointsContainer(transformed_plan,
                                 trajectory_global_plan_viapoint_sep_, trajectory_global_plan_goal_sep_);
        via_points_vec_.push_back(Eigen::Vector3d(goal_x, goal_y, goal_th));

        local_plan_vec_.push_back(transformed_plan.back());

        double lethal_distance = checkCollision(transformed_plan, trajectory_max_global_plan_lookahead_dist_);
        dec_ratio_ = std::min(lethal_distance / robot_dec_dist_, 1.0);

        Eigen::Quaterniond quat_world_robot(robot_pose_.pose.orientation.w, robot_pose_.pose.orientation.x,
                                            robot_pose_.pose.orientation.y, robot_pose_.pose.orientation.z);
        Eigen::Matrix3d rot_mat_world_robot = quat_world_robot.toRotationMatrix();
        Eigen::Translation3d trans_world_robot(robot_pose_.pose.position.x, robot_pose_.pose.position.y, 0.0);
        Eigen::Affine3d T_world_robot = trans_world_robot * rot_mat_world_robot;

        Eigen::Vector3d euler_world_target(via_points_vec_[0][2], 0, 0);
        Eigen::Matrix3d rot_mat_world_target;
        rot_mat_world_target = Eigen::AngleAxisd(euler_world_target(0), Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(euler_world_target(1), Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(euler_world_target(2), Eigen::Vector3d::UnitX());
        Eigen::Translation3d trans_world_target(via_points_vec_[0][0], via_points_vec_[0][1], 0.0);
        Eigen::Affine3d T_world_target = trans_world_target * rot_mat_world_target;

        Eigen::Affine3d T_robot_target = T_world_robot.inverse() * T_world_target;
        Eigen::Matrix3d rot_mat_robot_target = T_robot_target.rotation();
        Eigen::Vector3d euler_robot_target = R2ypr(rot_mat_robot_target);
        double dx1 = T_robot_target.matrix()(0, 3);
        double dy1 = T_robot_target.matrix()(1, 3);
        double dyaw1 = euler_robot_target(0);

        double rho, alpha, phi, v, omega;
        poseError(dx1, dy1, dyaw1, rho, alpha, phi);
        homingControl(rho, alpha, phi, v, omega);

        if (robot_min_turn_radius_ > 0.0)
        {
            double omega_max = fabs(v / robot_min_turn_radius_);
            omega = clip(omega, omega_max * (-1.0), omega_max);
        }
        else
        {
            double d_atan = std::atan2(dy1, dx1);
            double d_atan_phi = fabs(d_atan - phi);
            if ((xy_reached_) or
                (robot_turn_around_priority_ and fabs(phi) > 0.75 and
                 (d_atan_phi < 0.5 or fmod(d_atan_phi, M_PI) < 0.5 or fmod(d_atan_phi, M_PI) > 2.6)))
            {
                v = 0;
                omega = clip(phi, robot_max_vel_theta_ * (-1.0), robot_max_vel_theta_);
            }
        }

        if (lethal_distance < robot_stop_dist_)
        {
            v = 0;
            omega = 0;
        }

        cmd_vel.twist.linear.x = v;
        cmd_vel.twist.angular.z = omega;

        visualization_->publishViaPoints(via_points_vec_);
        visualization_->publishViaPoints(collision_points_, "CollsionPoints", visualization_->toColorMsg(1.0, 1.0, 0.65, 0.0));
        local_plan_.poses = local_plan_vec_;
        // local_plan_.header.frame_id = global_plan_.header.frame_id;
        local_plan_.header.frame_id = global_frame_;
        local_plan_.header.stamp = rclcpp::Time();
        visualization_->publishLocalPlan(local_plan_);
        visualization_->publishGlobalPlan(global_plan_);

        return cmd_vel;
    }

    bool HomingLocalPlanner::pruneGlobalPlan(const std::shared_ptr<tf2_ros::Buffer> &tf, const geometry_msgs::msg::PoseStamped &global_pose,
                                             std::vector<geometry_msgs::msg::PoseStamped> &global_plan, double dist_behind_robot)
    {
        if (global_plan.empty())
            return true;
        try
        {
            geometry_msgs::msg::TransformStamped global_to_plan_transform = tf->lookupTransform(global_plan_.header.frame_id, global_pose.header.frame_id, tf2::TimePointZero);

            geometry_msgs::msg::PoseStamped robot;
            tf2::doTransform(global_pose, robot, global_to_plan_transform); // global_pose w.r.t odom, robot w.r.t map

            double dist_thresh_sq = dist_behind_robot * dist_behind_robot;

            // iterate plan until a pose close the robot is found
            std::vector<geometry_msgs::msg::PoseStamped>::iterator it = global_plan.begin();
            std::vector<geometry_msgs::msg::PoseStamped>::iterator erase_end = it;
            while (it != global_plan.end())
            {
                double dx = robot.pose.position.x - it->pose.position.x;
                double dy = robot.pose.position.y - it->pose.position.y;
                double dist_sq = dx * dx + dy * dy;
                if (dist_sq < dist_thresh_sq)
                {
                    erase_end = it;
                    break;
                }
                ++it;
            }
            if (erase_end == global_plan.end())
                return false;

            if (erase_end != global_plan.begin())
                global_plan.erase(global_plan.begin(), erase_end);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(logger_, "Cannot prune path since no transform is available: %s\n", ex.what());
            return false;
        }
        return true;
    }

    void HomingLocalPlanner::simplifyGlobalPlan(std::vector<geometry_msgs::msg::PoseStamped> &plan, double simplify_separation)
    {
        if (plan.empty())
            return;
        std::vector<geometry_msgs::msg::PoseStamped> simple_plan;
        simple_plan.push_back(plan[0]);
        std::size_t prev_idx = 0;
        for (std::size_t i = 1; i < plan.size(); ++i)
        {
            if ((distancePoints2d(plan[prev_idx].pose.position, plan[i].pose.position) < simplify_separation) ||
                (distancePoints2d(plan.back().pose.position, plan[i].pose.position) < simplify_separation))
                continue;
            simple_plan.push_back(plan[i]);
            prev_idx = i;
        }

        simple_plan.push_back(plan.back());
        plan = simple_plan;
    }

    void HomingLocalPlanner::smoothPlan2d(std::vector<geometry_msgs::msg::PoseStamped> &global_plan)
    {
        if (global_plan.empty())
            return;
        std::vector<geometry_msgs::msg::PoseStamped> smoothed_global_plan(global_plan);
        double weight_data = 0.5;
        double weight_smooth = 1.0 - weight_data;
        double tolerance = 0.007;
        double change = tolerance;
        double aux1, aux2;
        while (change >= tolerance)
        {
            change = 0;
            for (std::size_t i = 1; i < global_plan.size() - 1; ++i)
            {
                aux1 = smoothed_global_plan[i].pose.position.x;
                smoothed_global_plan[i].pose.position.x += weight_data * (global_plan[i].pose.position.x - smoothed_global_plan[i].pose.position.x) + weight_smooth * (smoothed_global_plan[i - 1].pose.position.x + smoothed_global_plan[i + 1].pose.position.x - 2 * smoothed_global_plan[i].pose.position.x);

                aux2 = smoothed_global_plan[i].pose.position.y;
                smoothed_global_plan[i].pose.position.y += weight_data * (global_plan[i].pose.position.y - smoothed_global_plan[i].pose.position.y) + weight_smooth * (smoothed_global_plan[i - 1].pose.position.y + smoothed_global_plan[i + 1].pose.position.y - 2 * smoothed_global_plan[i].pose.position.y);
                change += (fabs(aux1 - smoothed_global_plan[i].pose.position.x) + fabs(aux2 - smoothed_global_plan[i].pose.position.y));
            }
        }
        global_plan = smoothed_global_plan;
    }

    void HomingLocalPlanner::updateViaPointsContainer(const std::vector<geometry_msgs::msg::PoseStamped> &transformed_plan, double min_separation_via, double min_separation_goal)
    {
        via_points_vec_.clear();
        local_plan_vec_.clear();

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = global_frame_;
        pose.header.stamp = rclcpp::Time();
        pose.pose = robot_pose_.pose;
        local_plan_vec_.push_back(pose);

        if (min_separation_via <= 0)
            return;

        std::size_t prev_idx = 0;
        double via_point_yaw;
        for (std::size_t i = 1; i < transformed_plan.size() - 1; ++i) // skip first one, since we do not need any point before the first min_separation [m]
        {
            // check separation to the previous via-point inserted
            if ((distancePoints2d(transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position) < min_separation_via) ||
                (distancePoints2d(transformed_plan.back().pose.position, transformed_plan[i].pose.position) < min_separation_goal) ||
                (distancePoints2d(robot_pose_.pose.position, transformed_plan[i].pose.position) < min_separation_via))
                continue;

            // add via-point
            via_point_yaw = std::atan2(transformed_plan[i + 1].pose.position.y - transformed_plan[i].pose.position.y,
                                       transformed_plan[i + 1].pose.position.x - transformed_plan[i].pose.position.x);

            via_points_vec_.push_back(Eigen::Vector3d(transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y, via_point_yaw));

            tf2::Quaternion quaternion;
            quaternion.setRPY(0.0, 0.0, via_point_yaw);

            geometry_msgs::msg::PoseStamped local_p = transformed_plan[i];
            local_p.pose.orientation = tf2::toMsg(quaternion);
            local_plan_vec_.push_back(local_p);
            prev_idx = i;
        }
    }

    // Transforms the global plan of the robot from the planner frame to the local frame (modified).
    bool HomingLocalPlanner::transformGlobalPlan(const std::shared_ptr<tf2_ros::Buffer> &tf,
                                                 const std::vector<geometry_msgs::msg::PoseStamped> &global_plan,
                                                 const geometry_msgs::msg::PoseStamped &global_pose,
                                                 const nav2_costmap_2d::Costmap2D &costmap,
                                                 const std::string &global_frame, double max_plan_length,
                                                 std::vector<geometry_msgs::msg::PoseStamped> &transformed_plan)
    {

        // const geometry_msgs::msg::PoseStamped &plan_pose = global_plan[0];
        transformed_plan.clear();
        try
        {
            if (global_plan.empty())
            {
                RCLCPP_WARN(logger_, "Received plan with zero length");
                return false;
            }

            // get plan_to_global_transform from plan frame to global_frame
            geometry_msgs::msg::TransformStamped plan_to_global_transform = tf->lookupTransform(global_frame,                 // odom
                                                                                                global_plan_.header.frame_id, // map
                                                                                                tf2::TimePointZero,
                                                                                                tf2::durationFromSec(0.5));
            // let's get the pose of the robot in the frame of the plan
            geometry_msgs::msg::PoseStamped robot_pose;
            tf->transform(global_pose, robot_pose, global_plan_.header.frame_id);

            // we'll discard points on the plan that are outside the local costmap
            double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                             costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
            dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                                    // located on the border of the local costmap

            int i = 0;
            double sq_dist_threshold = dist_threshold * dist_threshold;
            double sq_dist = 1e10;

            // we need to loop to a point on the plan that is within a certain distance of the robot
            bool robot_reached = false;
            for (int j = 0; j < (int)global_plan.size(); ++j)
            {
                double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
                double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
                double new_sq_dist = x_diff * x_diff + y_diff * y_diff;

                if (robot_reached && new_sq_dist > sq_dist)
                    break;

                if (new_sq_dist < sq_dist) // find closest distance
                {
                    sq_dist = new_sq_dist;
                    i = j;
                    if (sq_dist < 0.05)       // 2.5 cm to the robot; take the immediate local minima; if it's not the global
                        robot_reached = true; // minima, probably means that there's a loop in the path, and so we prefer this
                }
            }

            geometry_msgs::msg::PoseStamped newer_pose;
            double plan_length = 0; // check cumulative Euclidean distance along the plan

            // now we'll transform until points are outside of our distance threshold
            while (i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length <= 0 || plan_length <= max_plan_length))
            {
                const geometry_msgs::msg::PoseStamped &pose = global_plan[i];
                tf2::doTransform(pose, newer_pose, plan_to_global_transform);
                transformed_plan.push_back(newer_pose);

                double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
                double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
                sq_dist = x_diff * x_diff + y_diff * y_diff;

                // caclulate distance to previous pose
                if (i > 0 && max_plan_length > 0)
                    plan_length += distance_points2d(global_plan[i - 1].pose.position, global_plan[i].pose.position);

                ++i;
            }

            // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
            // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
            if (transformed_plan.empty())
            {
                tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);
                transformed_plan.push_back(newer_pose);
            }
        }
        catch (tf2::LookupException &ex)
        {
            RCLCPP_ERROR(logger_, "No Transform available Error: %s\n", ex.what());
            return false;
        }
        catch (tf2::ConnectivityException &ex)
        {
            RCLCPP_ERROR(logger_, "Connectivity Error: %s\n", ex.what());
            return false;
        }
        catch (tf2::ExtrapolationException &ex)
        {
            RCLCPP_ERROR(logger_, "Extrapolation Error: %s\n", ex.what());
            return false;
        }

        return true;
    }

    double HomingLocalPlanner::clip(double value, double lower, double upper)
    {
        if (value < lower)
            return lower;
        else if (value > upper)
            return upper;
        else
            return value;
    }

    void HomingLocalPlanner::cart2Pol(double x, double y, double &deg, double &dist)
    {
        dist = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        deg = std::atan2(y, x);
    }

    double HomingLocalPlanner::angDiff(double alpha, double beta)
    {
        double delta;
        delta = fmod((alpha - beta), 2.0 * M_PI);
        if (delta > M_PI)
            delta = delta - 2 * M_PI;
        else if (delta < M_PI * (-1.0))
            delta = delta + 2 * M_PI;
        return delta;
    }

    void HomingLocalPlanner::poseError(double dx, double dy, double dyaw, double &rho, double &alpha, double &phi)
    {
        cart2Pol(dx, dy, alpha, rho);
        alpha = angDiff(0, alpha);
        phi = dyaw;
    }

    void HomingLocalPlanner::homingControl(double rho, double alpha, double phi, double &v, double &omega)
    {
        v = optimization_k_rho_ * rho;
        if ((fabs(alpha) > M_PI * 0.5 + 0.1) or ((fabs(alpha) > M_PI * 0.5 - 0.1) and last_back_))
        {
            v = v * (-1.0);
            alpha = angDiff(alpha, M_PI);
            last_back_ = true;
        }
        else
            last_back_ = false;
        v = clip(v, robot_max_vel_x_ * (-1.0) * dec_ratio_, robot_max_vel_x_ * dec_ratio_);
        omega = optimization_k_alpha_ * alpha + optimization_k_phi_ * phi;
        omega = clip(omega, robot_max_vel_theta_ * (-1.0) * dec_ratio_, robot_max_vel_theta_ * dec_ratio_);
    }

    Eigen::Vector3d HomingLocalPlanner::R2ypr(const Eigen::Matrix3d &R)
    {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);
        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;
        return ypr;
    }

    double HomingLocalPlanner::checkCollision(const std::vector<geometry_msgs::msg::PoseStamped> &transformed_plan, double check_dist)
    {
        collision_points_.clear();
        collision_checker_.setCostmap(costmap_);
        double path_point_yaw;
        double plan_length = 0;
        double lethal_point_distance = 999;
        std::size_t i = 1;
        while (i < transformed_plan.size() && plan_length < check_dist)
        {
            path_point_yaw = std::atan2(transformed_plan[i].pose.position.y - transformed_plan[i - 1].pose.position.y,
                                        transformed_plan[i].pose.position.x - transformed_plan[i - 1].pose.position.x);
            double footprint_cost = collision_checker_.footprintCostAtPose(transformed_plan[i].pose.position.x,
                                                                           transformed_plan[i].pose.position.y, path_point_yaw, costmap_ros_->getRobotFootprint());
            plan_length += distance_points2d(transformed_plan[i].pose.position, transformed_plan[i - 1].pose.position);
            if (footprint_cost == nav2_costmap_2d::LETHAL_OBSTACLE) // 254
            {
                collision_points_.push_back(Eigen::Vector3d(transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y, path_point_yaw));
                if (plan_length < lethal_point_distance)
                {
                    lethal_point_distance = plan_length;
                    // double dist =  distance_point_to_polygon_2d(const Eigen::Vector2d &point, const Point2dContainer &vertices)
                    // boost::make_shared<PointObstacle>(transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y);
                }
            }
            i++;
        }
        return lethal_point_distance;
    }
}

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
    homing_local_planner::HomingLocalPlanner,
    nav2_core::Controller)