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

#ifndef ONE_EURO_FILTER_H_
#define ONE_EURO_FILTER_H_

#include <cmath>
#include <ros/ros.h>

namespace homing_local_planner
{
    class OneEuroFilter
    {
    public:
        OneEuroFilter(){};
        OneEuroFilter(ros::Time t0, double x0, double dx0, double min_cutoff, double beta,
                      double d_cutoff) : t_prev_(t0), x_prev_(x0), dx_prev_(dx0),
                                         min_cutoff_(min_cutoff), beta_(beta), d_cutoff_(d_cutoff){};
        ~OneEuroFilter(){};
        inline double exponentialSmoothing(double a, double x, double x_prev)
        {
            return a * x + (1 - a) * x_prev;
        };
        inline double smoothingFactor(double t_e, double cutoff)
        {
            double r = 2 * M_PI * cutoff * t_e;
            return r / (r + 1.0);
        };
        double filterCall(ros::Time t, double x)
        {
            double t_e = (t - t_prev_).toSec();
            double a_d = smoothingFactor(t_e, dx_prev_);
            double dx = (x - x_prev_) / t_e;
            double dx_hat = exponentialSmoothing(a_d, dx, dx_prev_);
            double cutoff = min_cutoff_ + beta_ * fabs(dx_hat);
            double a = smoothingFactor(t_e, cutoff);
            double x_hat;
            if (fabs(x) > 0.0001)
                x_hat = exponentialSmoothing(a, x, x_prev_);
            else
                x_hat = 0;
            x_prev_ = x_hat;
            dx_prev_ = dx_hat;
            t_prev_ = t;
            return x_hat;
        };

    private:
        double min_cutoff_, beta_, d_cutoff_, x_prev_, dx_prev_;
        ros::Time t_prev_;
    };
};
#endif