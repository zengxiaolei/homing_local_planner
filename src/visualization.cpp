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

#include <homing_local_planner/visualization.h>

namespace homing_local_planner
{
    HomingVisualization::HomingVisualization() : initialized_(false)
    {
    }

    HomingVisualization::HomingVisualization(ros::NodeHandle &nh, std::string frame_id) : initialized_(false), frame_id_(frame_id)
    {
        initialize(nh);
    }
    void HomingVisualization::initialize(ros::NodeHandle &nh)
    {
        if (initialized_)
            ROS_WARN("HomingVisualization already initialized. Reinitalizing...");

        homing_marker_pub_ = nh.advertise<visualization_msgs::Marker>("homing_markers", 100);
        local_plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 1);
        global_plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
        initialized_ = true;
    }
    void HomingVisualization::publishViaPoints(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &via_points,
                                               const std::string &ns, const std_msgs::ColorRGBA &color) const
    {
        if (via_points.empty() || printErrorWhenNotInitialized())
            return;

        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(2.0);

        for (std::size_t i = 0; i < via_points.size(); ++i)
        {
            geometry_msgs::Point point;
            point.x = via_points[i][0];
            point.y = via_points[i][1];
            point.z = 0;
            marker.points.push_back(point);
        }

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.color = color;

        homing_marker_pub_.publish(marker);
    }

    void HomingVisualization::publishLocalPlan(const std::vector<geometry_msgs::PoseStamped> &local_plan) const
    {
        if (printErrorWhenNotInitialized())
            return;
        base_local_planner::publishPlan(local_plan, local_plan_pub_);
    }

    void HomingVisualization::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped> &global_plan) const
    {
        if (printErrorWhenNotInitialized())
            return;
        base_local_planner::publishPlan(global_plan, global_plan_pub_);
    }

    bool HomingVisualization::printErrorWhenNotInitialized() const
    {
        if (!initialized_)
        {
            ROS_ERROR("HomingVisualization class not initialized. You must call initialize or an appropriate constructor");
            return true;
        }
        return false;
    }

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

    std_msgs::ColorRGBA HomingVisualization::toColorMsg(double a, double r, double g, double b)
    {
        std_msgs::ColorRGBA color;
        color.a = a;
        color.r = r;
        color.g = g;
        color.b = b;
        return color;
    }

}
