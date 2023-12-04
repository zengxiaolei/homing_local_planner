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

#include <pluginlib/class_list_macros.h>
#include "homing_local_planner/homing_local_planner_ros.h"

PLUGINLIB_EXPORT_CLASS(homing_local_planner::HomingLocalPlanner, nav_core::BaseLocalPlanner)

namespace homing_local_planner
{

    HomingLocalPlanner::HomingLocalPlanner() : tf_(NULL), dynamic_recfg_(NULL),
                                               goal_reached_(false), world_model_(NULL), initialized_(false)
    {
    }

    HomingLocalPlanner::~HomingLocalPlanner() {}

    void HomingLocalPlanner::reconfigureCB(HomingLocalPlannerReconfigureConfig &config, uint32_t level)
    {
        cfg_.reconfigure(config);
    }

    void HomingLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                                        costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            ros::NodeHandle nh("~/" + name);
            cfg_.loadRosParamFromNodeHandle(nh);
            obstacles_.reserve(500);

            // create robot footprint/contour model for optimization
            robot_model_ = getRobotFootprintFromParamServer(nh);

            tf_ = tf;
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            initialized_ = true;
            global_frame_ = costmap_ros_->getGlobalFrameID();
            robot_base_frame_ = costmap_ros_->getBaseFrameID();

            visualization_ = HomingVisualizationPtr(new HomingVisualization(nh, global_frame_));
            dynamic_recfg_ = boost::make_shared<dynamic_reconfigure::Server<HomingLocalPlannerReconfigureConfig>>(nh);
            dynamic_reconfigure::Server<HomingLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(&HomingLocalPlanner::reconfigureCB, this, _1, _2);
            dynamic_recfg_->setCallback(cb);
            world_model_ = new base_local_planner::CostmapModel(*costmap_);
        }
    }

    bool HomingLocalPlanner::setPlan(
        const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        if (!initialized_)
        {
            ROS_ERROR("homing_local_planner has not been initialized");
            return false;
        }
        // store the global plan
        global_plan_.clear();
        global_plan_ = orig_global_plan;
        goal_reached_ = false;
        xy_reached_ = false;
        last_back_ = false;
        omega_filter_.filterCall(ros::Time::now(), last_cmd_.angular.z);
        smoothPlan2d(global_plan_);
        visualization_->publishGlobalPlan(global_plan_);
        return true;
    }

    bool HomingLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        if (!initialized_)
        {
            ROS_ERROR("homing_local_planner has not been initialized");
            return false;
        }

        goal_reached_ = false;
        costmap_ros_->getRobotPose(robot_pose_);
        robot_pose_se2_ = PoseSE2(robot_pose_.pose);

        // prune global plan to cut off parts of the past (spatially before the robot)
        pruneGlobalPlan(*tf_, robot_pose_, global_plan_, cfg_.trajectory.global_plan_prune_distance);

        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        int goal_idx;
        geometry_msgs::TransformStamped tf_plan_to_global;
        if (!transformGlobalPlan(*tf_, global_plan_, robot_pose_, *costmap_, global_frame_, cfg_.trajectory.max_global_plan_lookahead_dist,
                                 transformed_plan, &goal_idx, &tf_plan_to_global))
        {
            ROS_WARN("Could not transform the global plan to the frame of the controller");
            return false;
        }
        if (transformed_plan.empty())
            return false;

        const geometry_msgs::PoseStamped &goal_point = transformed_plan.back();

        // we assume the global goal is the last point in the global plan
        const double goal_x = goal_point.pose.position.x;
        const double goal_y = goal_point.pose.position.y;
        const double goal_th = tf2::getYaw(goal_point.pose.orientation);
        double dx = goal_x - robot_pose_se2_.x();
        double dy = goal_y - robot_pose_se2_.y();
        double dyaw = fmod(goal_th - robot_pose_se2_.theta(), 2 * M_PI);

        if ((fabs(std::sqrt(dx * dx + dy * dy)) < cfg_.goal_tolerance.xy_goal_tolerance) && (fabs(dyaw) < cfg_.goal_tolerance.yaw_goal_tolerance))
            goal_reached_ = true;
        else if (fabs(std::sqrt(dx * dx + dy * dy)) < cfg_.goal_tolerance.xy_goal_tolerance)
            xy_reached_ = true;
        else if (xy_reached_ && (fabs(dyaw) < cfg_.goal_tolerance.yaw_goal_tolerance))
            goal_reached_ = true;

        updateViaPointsContainer(transformed_plan,
                                 cfg_.trajectory.global_plan_viapoint_sep, cfg_.trajectory.global_plan_goal_sep);
        via_points_.push_back(Eigen::Vector3d(goal_x, goal_y, goal_th));
        local_plan_.push_back(transformed_plan.back());

        obstacles_.clear();
        updateObstacleContainerWithCostmap();
        checkCollision(transformed_plan, 1);

        // homingcontrol2
        /*
        double v, omega;
        homingControl2(dx,dy,tf2::getYaw(robot_pose_.pose.orientation),  goal_th,v,omega);
        cmd_vel.linear.x =v;
        cmd_vel.angular.z = omega;
        */

        // homing control rst
        Eigen::Quaterniond quat_world_robot(robot_pose_.pose.orientation.w, robot_pose_.pose.orientation.x,
                                            robot_pose_.pose.orientation.y, robot_pose_.pose.orientation.z);
        Eigen::Matrix3d rot_mat_world_robot = quat_world_robot.toRotationMatrix();
        Eigen::Translation3d trans_world_robot(robot_pose_.pose.position.x, robot_pose_.pose.position.y, 0.0);
        Eigen::Affine3d T_world_robot = trans_world_robot * rot_mat_world_robot;

        Eigen::Vector3d euler_world_target(via_points_[0][2], 0, 0);
        Eigen::Matrix3d rot_mat_world_target;
        rot_mat_world_target = Eigen::AngleAxisd(euler_world_target(0), Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(euler_world_target(1), Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(euler_world_target(2), Eigen::Vector3d::UnitX());
        Eigen::Translation3d trans_world_target(via_points_[0][0], via_points_[0][1], 0.0);
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
        cmd_vel.linear.x = v;
        if (0)
            cmd_vel.angular.z = omega_filter_.filterCall(ros::Time::now(), omega);
        else
            cmd_vel.angular.z = omega;

        double d_atan = std::atan2(dy1, dx1);
        double d_atan_phi = fabs(d_atan - phi);
        if ((xy_reached_) or
            (cfg_.robot.turn_around_priority and fabs(phi) > 0.75 and
             (d_atan_phi < 0.5 or fmod(d_atan_phi, M_PI) < 0.5 or fmod(d_atan_phi, M_PI) > 2.6)))
        {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = clip(phi, cfg_.robot.max_vel_theta * (-1.0), cfg_.robot.max_vel_theta);
        }

        last_cmd_ = cmd_vel;
        visualization_->publishViaPoints(via_points_);
        visualization_->publishViaPoints(collision_points_, "CollsionPoints");
        visualization_->publishLocalPlan(local_plan_);
        visualization_->publishObstacles(obstacles_, costmap_->getResolution());
        visualization_->publishRobotFootprintModel(robot_pose_se2_, *robot_model_);
        return true;
    }

    bool HomingLocalPlanner::isGoalReached()
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        if (goal_reached_)
        {
            ROS_INFO("GOAL Reached!");
            // planner_->clearPlanner();
            return true;
        }
        return false;
    }

    bool HomingLocalPlanner::pruneGlobalPlan(const tf2_ros::Buffer &tf, const geometry_msgs::PoseStamped &global_pose, std::vector<geometry_msgs::PoseStamped> &global_plan, double dist_behind_robot)
    {
        if (global_plan.empty())
            return true;

        try
        {
            // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
            geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
            geometry_msgs::PoseStamped robot;
            tf2::doTransform(global_pose, robot, global_to_plan_transform);

            double dist_thresh_sq = dist_behind_robot * dist_behind_robot;

            // iterate plan until a pose close the robot is found
            std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
            std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
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
        catch (const tf::TransformException &ex)
        {
            ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
            return false;
        }
        return true;
    }

    void HomingLocalPlanner::smoothPlan2d(std::vector<geometry_msgs::PoseStamped> &global_plan)
    {
        if (global_plan.empty())
            return;
        simplifyGlobalPlan(global_plan, 0.03);
        std::vector<geometry_msgs::PoseStamped> smoothed_global_plan(global_plan);
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

    void HomingLocalPlanner::simplifyGlobalPlan(std::vector<geometry_msgs::PoseStamped> &plan, double simplify_separation)
    {
        if (plan.empty())
            return;
        std::vector<geometry_msgs::PoseStamped> simple_plan;
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

    void HomingLocalPlanner::updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped> &transformed_plan, double min_separation_via, double min_separation_goal)
    {
        via_points_.clear();
        local_plan_.clear();

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = global_frame_;
        pose.header.stamp = ros::Time::now();
        pose.pose = robot_pose_.pose;
        local_plan_.push_back(pose);

        if (min_separation_via <= 0)
            return;

        std::size_t prev_idx = 0;
        double via_point_yaw;
        for (std::size_t i = 1; i < transformed_plan.size(); ++i) // skip first one, since we do not need any point before the first min_separation [m]
        {
            // check separation to the previous via-point inserted
            if ((distancePoints2d(transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position) < min_separation_via) ||
                (distancePoints2d(transformed_plan.back().pose.position, transformed_plan[i].pose.position) < min_separation_goal) ||
                (distancePoints2d(robot_pose_.pose.position, transformed_plan[i].pose.position) < min_separation_via))
                continue;

            // add via-point
            via_point_yaw = std::atan2(transformed_plan[i].pose.position.y - transformed_plan[prev_idx].pose.position.y,
                                       transformed_plan[i].pose.position.x - transformed_plan[prev_idx].pose.position.x);
            via_points_.push_back(Eigen::Vector3d(transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y, via_point_yaw));
            geometry_msgs::PoseStamped local_plan_pose = transformed_plan[i];
            local_plan_pose.pose.orientation = tf::createQuaternionMsgFromYaw(via_point_yaw);
            local_plan_.push_back(local_plan_pose);
            prev_idx = i;
        }
    }
    bool HomingLocalPlanner::checkCollision(const std::vector<geometry_msgs::PoseStamped> &transformed_plan, double obst_dist)
    {
        collision_points_.clear();
        for (const ObstaclePtr &obst : obstacles_)
        {
            double dist = robot_model_->calculateDistance(robot_pose_se2_, obst.get());
        }

        std::size_t prev_idx = 0;
        double path_point_yaw;
        for (std::size_t i = 1; i < transformed_plan.size(); ++i)
        {
            path_point_yaw = std::atan2(transformed_plan[i].pose.position.y - transformed_plan[prev_idx].pose.position.y,
                                        transformed_plan[i].pose.position.x - transformed_plan[prev_idx].pose.position.x);
            PoseSE2 plan_se2 = PoseSE2(transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y, path_point_yaw);
            double footprint_cost = world_model_->footprintCost(transformed_plan[i].pose.position.x,
                                                                transformed_plan[i].pose.position.y, path_point_yaw, costmap_ros_->getUnpaddedRobotFootprint());
            if (footprint_cost == -1)
            {
                collision_points_.push_back(Eigen::Vector3d(transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y, path_point_yaw));
            }

            /*
            for (const ObstaclePtr &obst : obstacles_)
            {
                double dist = robot_model_->calculateDistance(plan_se2, obst.get());
                // if (dist < 0.5)
                //     std::cout << "dist: " << dist << std::endl;

                costmap_2d::transformFootprint(transformed_plan[i].pose.position.x,
                                               transformed_plan[i].pose.position.y,
                                               path_point_yaw,
                                               costmap_ros_->getUnpaddedRobotFootprint(), transformed_footprint_);

                geometry_msgs::Point32 point;
                point.x = obst->getCentroid()[0];
                point.y = obst->getCentroid()[1];
                point.z = 0.0;
                // base_local_planner::PointGrid::ptInPolygon(point, transformed_footprint_);
            }
            */
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
        v = cfg_.optimization.k_rho * rho;
        if ((fabs(alpha) > M_PI * 0.5 + 0.1) or ((fabs(alpha) > M_PI * 0.5 - 0.1) and last_back_))
        {
            v = v * (-1.0);
            alpha = angDiff(alpha, M_PI);
            last_back_ = true;
        }
        else
            last_back_ = false;
        v = clip(v, cfg_.robot.max_vel_x * (-1.0), cfg_.robot.max_vel_x);
        omega = cfg_.optimization.k_alpha * alpha + cfg_.optimization.k_phi * phi;
        omega = clip(omega, cfg_.robot.max_vel_theta * (-1.0), cfg_.robot.max_vel_theta);
    }

    void HomingLocalPlanner::homingControl2(double dx, double dy, double yaw, double yaw_goal, double &v, double &omega)
    {
        double rho = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        double alpha = fmod((std::atan2(dy, dx) - yaw + M_PI), 2.0 * M_PI);
        if (alpha < 0)
            alpha += 2 * M_PI;
        alpha = alpha - M_PI;

        double beta = fmod((yaw_goal - yaw - alpha + M_PI), 2 * M_PI);
        if (beta < 0)
            beta += 2 * M_PI;
        beta = beta - M_PI;
        // (9, 15, 3) Kp_rho, Kp_alpha, Kp_beta
        v = 9.0 * rho;
        omega = 15 * alpha - 3.0 * beta;
        if (fabs(alpha) > 0.5 * M_PI)
            v = -1.0 * v;

        v = clip(v, cfg_.robot.max_vel_x * (-1.0), cfg_.robot.max_vel_x);
        omega = clip(omega, cfg_.robot.max_vel_theta * (-1.0), cfg_.robot.max_vel_theta);
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

    void HomingLocalPlanner::updateObstacleContainerWithCostmap()
    {
        // Add costmap obstacles if desired
        // if (cfg_.obstacles.include_costmap_obstacles)
        if (true)
        {
            double theta = tf2::getYaw(robot_pose_.pose.orientation);
            Eigen::Vector2d robot_orient = Eigen::Vector2d(std::cos(theta), std::sin(theta));

            for (unsigned int i = 0; i < costmap_->getSizeInCellsX() - 1; ++i)
            {
                for (unsigned int j = 0; j < costmap_->getSizeInCellsY() - 1; ++j)
                {
                    if (costmap_->getCost(i, j) == costmap_2d::LETHAL_OBSTACLE)
                    {
                        Eigen::Vector2d obs;
                        costmap_->mapToWorld(i, j, obs.coeffRef(0), obs.coeffRef(1));

                        // check if obstacle is interesting (e.g. not far behind the robot)
                        Eigen::Vector2d obs_dir = obs - Eigen::Vector2d(robot_pose_.pose.position.x, robot_pose_.pose.position.y);
                        // if (obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_.obstacles.costmap_obstacles_behind_robot_dist)
                        //     continue;
                        if (obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > 1.0)
                            continue;

                        // if (obs_dir.norm() < 1.0)
                        //     std::cout << obs_dir.norm() << std::endl;

                        obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
                    }
                }
            }
        }
    }

    RobotFootprintModelPtr HomingLocalPlanner::getRobotFootprintFromParamServer(const ros::NodeHandle &nh)
    {
        std::string model_name;
        if (!nh.getParam("footprint_model/type", model_name))
        {
            ROS_INFO("No robot footprint model specified for trajectory optimization. Using point-shaped model.");
            return boost::make_shared<PointRobotFootprint>();
        }

        // circular
        if (model_name.compare("circular") == 0)
        {
            // get radius
            double radius;
            if (!nh.getParam("footprint_model/radius", radius))
            {
                ROS_ERROR_STREAM("Footprint model 'circular' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                                                                                                                          << "/footprint_model/radius' does not exist. Using point-model instead.");
                return boost::make_shared<PointRobotFootprint>();
            }
            ROS_INFO_STREAM("Footprint model 'circular' (radius: " << radius << "m) loaded for trajectory optimization.");
            return boost::make_shared<CircularRobotFootprint>(radius);
        }

        // polygon
        if (model_name.compare("polygon") == 0)
        {

            // check parameters
            XmlRpc::XmlRpcValue footprint_xmlrpc;
            if (!nh.getParam("footprint_model/vertices", footprint_xmlrpc))
            {
                ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                                                                                                                         << "/footprint_model/vertices' does not exist. Using point-model instead.");
                return boost::make_shared<PointRobotFootprint>();
            }
            // get vertices
            if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                try
                {
                    Point2dContainer polygon = makeFootprintFromXMLRPC(footprint_xmlrpc, "/footprint_model/vertices");
                    ROS_INFO_STREAM("Footprint model 'polygon' loaded for trajectory optimization.");
                    return boost::make_shared<PolygonRobotFootprint>(polygon);
                }
                catch (const std::exception &ex)
                {
                    ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization: " << ex.what() << ". Using point-model instead.");
                    return boost::make_shared<PointRobotFootprint>();
                }
            }
            else
            {
                ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                                                                                                                         << "/footprint_model/vertices' does not define an array of coordinates. Using point-model instead.");
                return boost::make_shared<PointRobotFootprint>();
            }
        }

        // otherwise
        ROS_WARN_STREAM("Unknown robot footprint model specified with parameter '" << nh.getNamespace() << "/footprint_model/type'. Using point model instead.");
        return boost::make_shared<PointRobotFootprint>();
    }

    Point2dContainer HomingLocalPlanner::makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue &footprint_xmlrpc, const std::string &full_param_name)
    {
        // Make sure we have an array of at least 3 elements.
        if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
            footprint_xmlrpc.size() < 3)
        {
            ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
                      full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
            throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
                                     "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
        }

        Point2dContainer footprint;
        Eigen::Vector2d pt;

        for (int i = 0; i < footprint_xmlrpc.size(); ++i)
        {
            // Make sure each element of the list is an array of size 2. (x and y coordinates)
            XmlRpc::XmlRpcValue point = footprint_xmlrpc[i];
            if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
                point.size() != 2)
            {
                ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
                          "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                          full_param_name.c_str());
                throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
                                         "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
            }

            pt.x() = getNumberFromXMLRPC(point[0], full_param_name);
            pt.y() = getNumberFromXMLRPC(point[1], full_param_name);

            footprint.push_back(pt);
        }
        return footprint;
    }

    double HomingLocalPlanner::getNumberFromXMLRPC(XmlRpc::XmlRpcValue &value, const std::string &full_param_name)
    {
        // Make sure that the value we're looking at is either a double or an int.
        if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
            value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
        {
            std::string &value_string = value;
            ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
                      full_param_name.c_str(), value_string.c_str());
            throw std::runtime_error("Values in the footprint specification must be numbers");
        }
        return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
    }

    bool HomingLocalPlanner::transformGlobalPlan(const tf2_ros::Buffer &tf, const std::vector<geometry_msgs::PoseStamped> &global_plan,
                                                 const geometry_msgs::PoseStamped &global_pose, const costmap_2d::Costmap2D &costmap, const std::string &global_frame, double max_plan_length,
                                                 std::vector<geometry_msgs::PoseStamped> &transformed_plan, int *current_goal_idx, geometry_msgs::TransformStamped *tf_plan_to_global) const
    {
        const geometry_msgs::PoseStamped &plan_pose = global_plan[0];
        transformed_plan.clear();
        try
        {
            if (global_plan.empty())
            {
                ROS_ERROR("Received plan with zero length");
                *current_goal_idx = 0;
                return false;
            }

            // get plan_to_global_transform from plan frame to global_frame
            geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp,
                                                                                          plan_pose.header.frame_id, ros::Duration(0.5));

            // let's get the pose of the robot in the frame of the plan
            geometry_msgs::PoseStamped robot_pose;
            tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);

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

            geometry_msgs::PoseStamped newer_pose;
            double plan_length = 0; // check cumulative Euclidean distance along the plan

            // now we'll transform until points are outside of our distance threshold
            while (i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length <= 0 || plan_length <= max_plan_length))
            {
                const geometry_msgs::PoseStamped &pose = global_plan[i];
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

                // Return the index of the current goal point (inside the distance threshold)
                if (current_goal_idx)
                    *current_goal_idx = int(global_plan.size()) - 1;
            }
            else
            {
                // Return the index of the current goal point (inside the distance threshold)
                if (current_goal_idx)
                    *current_goal_idx = i - 1; // subtract 1, since i was increased once before leaving the loop
            }

            // Return the transformation from the global plan to the global planning frame if desired
            if (tf_plan_to_global)
                *tf_plan_to_global = plan_to_global_transform;
        }
        catch (tf::LookupException &ex)
        {
            ROS_ERROR("No Transform available Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException &ex)
        {
            ROS_ERROR("Connectivity Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException &ex)
        {
            ROS_ERROR("Extrapolation Error: %s\n", ex.what());
            if (global_plan.size() > 0)
                ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

            return false;
        }

        return true;
    }
}