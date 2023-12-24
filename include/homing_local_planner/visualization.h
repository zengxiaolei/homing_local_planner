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

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include "Eigen/Core"
#include <string>
// boost
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <nav_msgs/Path.h>
#include <base_local_planner/goal_functions.h>
#include <homing_local_planner/obstacles.h>
#include <homing_local_planner/pose_se2.h>

namespace homing_local_planner
{
    class HomingVisualization
    {
    public:
        HomingVisualization();
        HomingVisualization(ros::NodeHandle &nh, std::string frame_id);
        void initialize(ros::NodeHandle &nh);
        void publishViaPoints(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &via_points,
                              const std::string &ns = "ViaPoints", const std_msgs::ColorRGBA &color = toColorMsg(1.0, 0.0, 0.0, 1.0)) const;
        void publishLocalPlan(const std::vector<geometry_msgs::PoseStamped> &local_plan) const;
        void publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped> &global_plan) const;
        void publishObstacles(const ObstContainer &obstacles, double scale = 0.1) const;

        /**
         * @brief Helper function to generate a color message from single values
         * @param a Alpha value
         * @param r Red value
         * @param g Green value
         * @param b Blue value
         * @return Color message
         */
        static std_msgs::ColorRGBA toColorMsg(double a, double r, double g, double b);

    protected:
        bool printErrorWhenNotInitialized() const;
        ros::Publisher homing_marker_pub_; //!< Publisher for visualization markers
        ros::Publisher local_plan_pub_;    //!< Publisher for the local plan
        ros::Publisher global_plan_pub_;

        bool initialized_;
        std::string frame_id_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    //! Abbrev. for shared instances of the HomingVisualization
    typedef boost::shared_ptr<HomingVisualization> HomingVisualizationPtr;
}
#endif /* VISUALIZATION_H_ */