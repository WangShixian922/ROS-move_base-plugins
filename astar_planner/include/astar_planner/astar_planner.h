/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * Author: WANG SHIXIAN
 *********************************************************************/

#ifndef ASTAR_PLANNER_H_
#define ASTAR_PLANNER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <string>
#include "Astar.h"

namespace astar_planner
{
  /**
   * @class AstarPlanner
   * @brief Provides a ROS wrapper for the global planner that uses the A* algorithm
   */
  class AstarPlanner : public nav_core::BaseGlobalPlanner
  {
  public:
    /**
     * @brief  Constructor for the AstarPlanner
     */
    AstarPlanner();
    /**
     * @brief  Constructor for the AstarPlanner
     * @param  name The name of this planner
     * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     */
    AstarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

    /**
     * @brief Destructor
     */
    ~AstarPlanner() {}

    /**
     * @brief  Initialization function for the AstarPlanner
     * @param  name The name of this planner
     * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     */
    void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

    /**
     * @brief Given a goal pose in the world, compute a plan
     * @param start The start pose
     * @param goal The goal pose
     * @param plan The plan... filled by the planner
     * @return True if a valid plan was found, false otherwise
     */
    bool makePlan(const geometry_msgs::PoseStamped &start,
                  const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan);

    /**
     * @brief  Publish a path for visualization purposes
     */
    void publishPlan(const std::vector<geometry_msgs::PoseStamped> &path, double r, double g, double b, double a);

  protected:
    boost::shared_ptr<astar_planner::AStar> planner_;
    ros::Publisher plan_pub_;

  private:
    void mapToWorld(double mx, double my, double &wx, double &wy);
    costmap_2d::Costmap2DROS *costmap_ros_;
    costmap_2d::Costmap2D *costmap_;
    bool initialized_, allow_unknown_;
    boost::mutex mutex_;
  };
};
#endif