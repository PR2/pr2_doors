/*********************************************************************
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
 *   * Neither the name of Willow Garage nor the names of its
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
 *********************************************************************/

/* Author: Wim Meeusen */

#ifndef ACTION_MOVE_BASE_DOOR_H
#define ACTION_MOVE_BASE_DOOR_H

#include <ros/ros.h>
#include <door_msgs/Door.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <kdl/frames.hpp>
#include <door_msgs/DoorGoal.h>
#include <door_msgs/DoorAction.h>
#include <actionlib/server/simple_action_server.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Point.h>


namespace door_handle_detector{


class MoveBaseDoorAction
{
public:
  MoveBaseDoorAction(tf::TransformListener& tf);
  ~MoveBaseDoorAction();

  void execute(const door_msgs::DoorGoalConstPtr& goal);

private:
  geometry_msgs::Point toPoint(const tf::Vector3& pnt);
  geometry_msgs::Vector3 toVector(const tf::Vector3& pnt);
  std::vector<geometry_msgs::Point> getOrientedFootprint(const tf::Vector3 pos, double theta_cost);

  tf::TransformListener& tf_;
  costmap_2d::Costmap2DROS costmap_ros_;
  costmap_2d::Costmap2D costmap_;
  base_local_planner::CostmapModel costmap_model_;
  ros::Publisher base_pub_;

  actionlib::SimpleActionServer<door_msgs::DoorAction> action_server_;
  std::vector<double> search_pattern_forward_, search_pattern_sideways_;
  std::vector<geometry_msgs::Point> footprint_;
};

}

#endif
