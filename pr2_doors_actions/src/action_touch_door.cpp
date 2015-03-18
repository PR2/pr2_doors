/*********************************************************************
n * Software License Agreement (BSD License)
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
#include "pr2_doors_actions/action_touch_door.h"

using namespace tf;
using namespace KDL;
using namespace ros;
using namespace std;
using namespace door_handle_detector;
using namespace pr2_doors_common;
using namespace actionlib;

static const string fixed_frame = "odom_combined";
static const double touch_dist = 0.65;
static const double gripper_effort = 100;


TouchDoorAction::TouchDoorAction(tf::TransformListener& tf) : 
  tf_(tf),
  action_server_(ros::NodeHandle(), 
		 "touch_door", 
		 boost::bind(&TouchDoorAction::execute, this, _1)),
  gripper_action_client_("r_gripper_controller/gripper_action", true)
{};


TouchDoorAction::~TouchDoorAction()
{};


void TouchDoorAction::execute(const door_msgs::DoorGoalConstPtr& goal)
{
  ROS_INFO("TouchDoorAction: execute");
 
  // transform door message to time fixed frame
  door_msgs::Door goal_tr;
  if (!transformTo(tf_, fixed_frame, goal->door, goal_tr, fixed_frame)){
    ROS_ERROR("Could not tranform door message from '%s' to '%s' at time %f",
	      goal->door.header.frame_id.c_str(), fixed_frame.c_str(), goal->door.header.stamp.toSec());
    action_server_.setPreempted();
    return;
  }

  // check for preemption
  if (action_server_.isPreemptRequested()){
    ROS_ERROR("TouchDoorAction: preempted");
    action_server_.setPreempted();
    return;
  }

  // close the gripper 
  pr2_controllers_msgs::Pr2GripperCommandGoal gripper_msg;
  gripper_msg.command.position = 0.0;
  gripper_msg.command.max_effort = gripper_effort;
  if (gripper_action_client_.sendGoalAndWait(gripper_msg, ros::Duration(10.0), ros::Duration(5.0)) != SimpleClientGoalState::SUCCEEDED){
    ROS_ERROR("TouchDoorAction: gripper failed to close");
    action_server_.setPreempted();
    return;
  }

  // get gripper position that is feasable for the robot arm
  StampedTransform shoulder_pose; tf_.lookupTransform(goal_tr.header.frame_id, "r_shoulder_pan_link", Time(), shoulder_pose);
  // this check does not make sense because we're not tracking the door angle while opinging
  //double angle = getNearestDoorAngle(shoulder_pose, goal_tr, 0.75, touch_dist);
  //if (fabs(angle) > fabs(getDoorAngle(goal_tr))){
  //  ROS_ERROR("Door touch pose for the gripper is inside the door");
  //  return robot_actions::ABORTED;
  //}
  Stamped<Pose> gripper_pose = getGripperPose(goal_tr, getNearestDoorAngle(shoulder_pose, goal_tr, 0.75, touch_dist), touch_dist);
  gripper_pose.stamp_ = Time::now();
  poseStampedTFToMsg(gripper_pose, req_moveto.pose);
  //req_moveto.tolerance.vel.x = 0.1;
  //req_moveto.tolerance.vel.y = 0.1;
  //req_moveto.tolerance.vel.z = 0.1;

  // move gripper in front of door
  ROS_INFO("move to touch the door at distance %f from hinge", touch_dist);
  if (!ros::service::call("r_arm_constraint_cartesian_trajectory_controller/move_to", req_moveto, res_moveto)){
    ROS_ERROR("move_to command failed");
    action_server_.setPreempted();
    return;
  }

  action_result_.door = goal->door;
  action_server_.setSucceeded(action_result_);  
}


