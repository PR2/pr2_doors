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

/* Author: Wim Meeussen */

#include <pr2_doors_common/door_functions.h>
#include "pr2_doors_actions/action_check_path.h"

using namespace tf;
using namespace KDL;
using namespace ros;
using namespace std;
using namespace door_handle_detector;
using namespace pr2_doors_common;

static const string fixed_frame = "map";


CheckPathAction::CheckPathAction(tf::TransformListener& tf) : 
  tf_(tf),
  action_server_(ros::NodeHandle(), 
		 "check_path", 
		 boost::bind(&CheckPathAction::execute, this, _1))
{};


CheckPathAction::~CheckPathAction()
{};



void CheckPathAction::execute(const door_msgs::CheckPathGoalConstPtr& goal)
{
  ROS_INFO("CheckPathAction: execute");

  // transform goal message to map frame
  geometry_msgs::PoseStamped goal_tr;
  ros::Duration timeout(3.0);
  if (!tf_.waitForTransform(goal->goal_pose.header.frame_id, fixed_frame, goal->goal_pose.header.stamp, timeout)){
    ROS_ERROR("cannot transform goal from %s to %s at time %f", goal->goal_pose.header.frame_id.c_str(), fixed_frame.c_str(), goal->goal_pose.header.stamp.toSec());
    action_server_.setPreempted();
    return;
  }
  tf_.transformPose(fixed_frame, goal->goal_pose, goal_tr);

  // check how far goal point is from current robot pose
  if (!tf_.waitForTransform(goal_tr.header.frame_id, "base_footprint", Time(), Duration(1.0))){
    ROS_ERROR("cannot get transform from %s to %s at time %f", goal_tr.header.frame_id.c_str(), string("base_footprint").c_str(), 0.0);
    action_server_.setPreempted();
    return;
  }
  tf::Stamped<tf::Point> goal_point;
  goal_point[0] = goal_tr.pose.position.x;  goal_point[1] = goal_tr.pose.position.y;  goal_point[2] = 0;
  goal_point.frame_id_ = goal_tr.header.frame_id;
  goal_point.stamp_ = Time();
  tf_.transformPoint("base_footprint", goal_point, goal_point);
  double length_straight = goal_point.length();
  ROS_INFO("Try to find path to (%f %f %f), which is %f [m] from the current robot pose.", 
	   goal_tr.pose.position.x, goal_tr.pose.position.y, goal_tr.pose.position.z, length_straight);
  
  
  ROS_INFO("call planner to find path");
  req_plan.goal = goal_tr;
  req_plan.tolerance = 0.0;
  Duration timeout_check_path(5.0);
  Time start_time = Time::now();
  while (ros::ok() && !action_server_.isPreemptRequested()){
    // check for timeout
    if (Time::now() > start_time + timeout_check_path){
      ROS_ERROR("Timeout checking paths. Did not find a path.");
      action_result_.path_found = false;
      action_server_.setSucceeded(action_result_);  
      return;
    }
    // find path
    if (!ros::service::call("move_base_node/make_plan", req_plan, res_plan))
      ROS_ERROR("Service for check path is not available");
    else if (res_plan.plan.poses.size() >= 2){
      ROS_INFO("Found possible path");
      // calculate length of path
      double length = 0;
      for (unsigned int i=0; i<res_plan.plan.poses.size()-1; i++){
	length += sqrt( ((res_plan.plan.poses[i].pose.position.x - res_plan.plan.poses[i+1].pose.position.x)*
			 (res_plan.plan.poses[i].pose.position.x - res_plan.plan.poses[i+1].pose.position.x))+
			((res_plan.plan.poses[i].pose.position.y - res_plan.plan.poses[i+1].pose.position.y)*
			 (res_plan.plan.poses[i].pose.position.y - res_plan.plan.poses[i+1].pose.position.y)));
      }
      if (length < 2.0 * length_straight && length < 5.0){
	ROS_INFO("received path from planner of length %f", length);
	action_result_.path_found = true;
	action_server_.setSucceeded(action_result_);  
	return;
      }
    }
    Duration(1.0).sleep();
  }
  ROS_ERROR("CheckPathAction: preempted");
  action_server_.setPreempted();
};

