/*********************************************************************
 *
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2023
 *
 * Author: Lei ZENG
 *********************************************************************/

#include <homing_local_planner/visualization.hpp>

namespace homing_local_planner
{
    HomingVisualization::HomingVisualization() : initialized_(false)
    {
    }

    HomingVisualization::HomingVisualization(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node, std::string frame_id) : initialized_(false), frame_id_(frame_id)
    {
        initialize(node);
    }
    void HomingVisualization::initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
    {
        if (initialized_)
            std::cout << "HomingVisualization already initialized. Reinitalizing...";

        global_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("homing_global_plan", 1);
        local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("homing_local_plan", 1);
        homing_marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("homing_markers", 1);

        initialized_ = true;
    }

    void HomingVisualization::activate()
    {

        global_plan_pub_->on_activate();
        local_plan_pub_->on_activate();
        homing_marker_pub_->on_activate();
    }

    void HomingVisualization::deactivate()
    {

        global_plan_pub_->on_deactivate();
        local_plan_pub_->on_deactivate();
        homing_marker_pub_->on_deactivate();
    }

    void HomingVisualization::cleanup()
    {
        global_plan_pub_.reset();
        local_plan_pub_.reset();
        homing_marker_pub_.reset();
    }

    void HomingVisualization::publishViaPoints(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &via_points,
                                               const std::string &ns, const std_msgs::msg::ColorRGBA &color) const
    {
        if (via_points.empty())
            return;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = rclcpp::Time();
        marker.ns = ns;
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration(2.0);

        for (std::size_t i = 0; i < via_points.size(); ++i)
        {
            geometry_msgs::msg::Point point;
            point.x = via_points[i][0];
            point.y = via_points[i][1];
            point.z = 0;
            marker.points.push_back(point);
        }

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.color = color;

        homing_marker_pub_->publish(marker);
    }

    void HomingVisualization::publishLocalPlan(const nav_msgs::msg::Path &local_plan) const
    {
        local_plan_pub_->publish(local_plan);
    }

    void HomingVisualization::publishGlobalPlan(const nav_msgs::msg::Path &global_plan) const
    {
        global_plan_pub_->publish(global_plan);
    }

    void HomingVisualization::publishGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped> &global_plan, std::string frame_id) const
    {
        nav_msgs::msg::Path pts_msg;
        pts_msg.header.frame_id = frame_id;
        pts_msg.header.stamp = rclcpp::Time();
        for (auto p : global_plan)
        {
            pts_msg.poses.push_back(p);
        }
        global_plan_pub_->publish(pts_msg);
    }

    /*
    void HomingVisualization::publishObstacles(const ObstContainer &obstacles, double scale) const
    {
        if (obstacles.empty() || printErrorWhenNotInitialized())
            return;

        // Visualize point obstacles
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = ros::Time::now();
            marker.ns = "PointObstacles";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::POINTS;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration(2.0);

            for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
            {
                boost::shared_ptr<PointObstacle> pobst = boost::dynamic_pointer_cast<PointObstacle>(*obst);
                if (!pobst)
                    continue;

                if (true)
                {
                    geometry_msgs::Point point;
                    point.x = pobst->x();
                    point.y = pobst->y();
                    point.z = 0;
                    marker.points.push_back(point);
                }
            }

            marker.scale.x = scale;
            marker.scale.y = scale;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            homing_marker_pub_.publish(marker);
        }
    }
    */

    std_msgs::msg::ColorRGBA HomingVisualization::toColorMsg(double a, double r, double g, double b)
    {
        std_msgs::msg::ColorRGBA color;
        color.a = a;
        color.r = r;
        color.g = g;
        color.b = b;
        return color;
    }

}
