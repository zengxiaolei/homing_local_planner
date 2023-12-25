/*********************************************************************
 *
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2023
 *
 * Author: Lei ZENG
 *********************************************************************/

#ifndef VISUALIZATION_HPP_
#define VISUALIZATION_HPP_

#include "Eigen/Core"

// boost
// #include <boost/graph/adjacency_list.hpp>
// #include <boost/graph/graph_traits.hpp>

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
// #include "nav_msgs/msg/Path.hpp"

#include "nav2_core/controller.hpp"

#include <boost/graph/adjacency_list.hpp>

namespace homing_local_planner
{
    class HomingVisualization
    {
    public:
        HomingVisualization();
        HomingVisualization(
            const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
            std::string frame_id);
        void initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node);
        void activate();
        void deactivate();
        void cleanup();
        void publishViaPoints(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &via_points,
                              const std::string &ns = "ViaPoints", const std_msgs::msg::ColorRGBA &color = toColorMsg(1.0, 0.0, 0.0, 1.0)) const;
        void publishLocalPlan(const nav_msgs::msg::Path &local_plan) const;
        void publishGlobalPlan(const nav_msgs::msg::Path &global_plan) const;
        void publishGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped> &global_plan, std::string frame_id) const;

        // void publishObstacles(const ObstContainer &obstacles, double scale = 0.1) const;
        static std_msgs::msg::ColorRGBA toColorMsg(double a, double r, double g, double b);

    protected:
        bool initialized_;
        std::string frame_id_;

        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_plan_pub_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_plan_pub_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>> homing_marker_pub_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    typedef boost::shared_ptr<HomingVisualization> HomingVisualizationPtr;
}
#endif /* VISUALIZATION_H_ */