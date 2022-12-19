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

#include <angles/angles.h>
#include "../include/astar_planner/astar_planner.h"
#include <pluginlib/class_list_macros.hpp>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(astar_planner::AstarPlanner, nav_core::BaseGlobalPlanner)

namespace astar_planner
{

  AstarPlanner::AstarPlanner()
      : costmap_ros_(NULL), costmap_(NULL), planner_(), initialized_(false) {}

  AstarPlanner::AstarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
      : costmap_ros_(NULL), costmap_(NULL), planner_(), initialized_(false)
  {
    initialize(name, costmap_ros);
  }

  void AstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    if (!initialized_)
    {
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
      planner_ = boost::shared_ptr<astar_planner::AStar>(new astar_planner::AStar(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY()));
      ros::NodeHandle private_nh("~/" + name);
      plan_pub_ = private_nh.advertise<nav_msgs::Path>("astar_plan", 1);
      private_nh.param("allow_unknown", allow_unknown_, true);
      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  bool AstarPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                              const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_)
    {
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    costmap_ = costmap_ros_->getCostmap();
    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int mx, my;
    if (!costmap_->worldToMap(wx, wy, mx, my))
    {
      ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
      return false;
    }
    ROS_INFO("start world to map [%f,%f],[%d,%d]", wx, wy, mx, my);

    // make sure to resize the underlying array that Navfn uses
    planner_->setMapSize(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

    int map_start[2];
    map_start[0] = mx;
    map_start[1] = my;

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, mx, my))
    {
      ROS_WARN_THROTTLE(1.0, "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
      return false;
    }
    // ROS_INFO("goal world to map [%f,%f],[%d,%d]", wx, wy, mx, my);

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;
    // ROS_INFO("map_start is [%d,%d]", map_start[0], map_start[1]);
    // ROS_INFO("map_goal is [%d,%d]", map_goal[0], map_goal[1]);

    planner_->expand(map_start, map_goal);
    planner_->waypoint = start;
    // ROS_INFO("frame id of start: %s", start.header.frame_id.c_str());
    planner_->generatePath(plan);
    std::reverse(plan.begin(), plan.end());
    for (std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin(); it != plan.end(); it++)
    {
      double world_x, world_y;
      mapToWorld((*it).pose.position.x, (*it).pose.position.y, world_x, world_y);
      (*it).header.stamp = ros::Time::now();
      (*it).pose.position.x = world_x;
      (*it).pose.position.y = world_y;
      // (*it).pose.position.z = 0.0;
      // (*it).pose.orientation.x = 0.0;
      // (*it).pose.orientation.y = 0.0;
      // (*it).pose.orientation.z = 0.0;
      // (*it).pose.orientation.w = 1.0;
    }
    // plan.push_back(start);
    // plan.push_back(goal);
    // ROS_INFO("3.the size of plan is %d", plan.size());
    publishPlan(plan, 0.0, 1.0, 0.0, 0.0);
    return (!plan.empty());
  }

  void AstarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path, double r, double g, double b, double a)
  {
    if (!initialized_)
    {
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    // create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if (path.empty())
    {
      // still set a valid frame so visualization won't hit transform issues
      gui_path.header.frame_id = "map";
      gui_path.header.stamp = ros::Time::now();
    }
    else
    {
      gui_path.header.frame_id = path[0].header.frame_id;
      gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++)
    {
      gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
  }

  void AstarPlanner::mapToWorld(double mx, double my, double &wx, double &wy)
  {
    wx = costmap_->getOriginX() + mx * costmap_->getResolution();
    wy = costmap_->getOriginY() + my * costmap_->getResolution();
  }

};
