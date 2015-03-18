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
 *  FOR A PARTICULAR PURPOSEARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ******************************p***************************************/

/* Author: Wim Meeussen */

#include <pr2_doors_common/door_functions.h>
#include "pr2_doors_actions/action_grasp_handle.h"

using namespace tf;
using namespace KDL;
using namespace ros;
using namespace std;
using namespace door_handle_detector;
using namespace pr2_doors_common;
using namespace actionlib;

static const string fixed_frame = "odom_combined";
static const double gripper_effort = 100;



GraspHandleAction::GraspHandleAction(tf::TransformListener& tf) : 
  tf_(tf),
  action_server_(ros::NodeHandle(), 
		 "grasp_handle", 
		 boost::bind(&GraspHandleAction::execute, this, _1)),
  gripper_action_client_("r_gripper_controller/gripper_action", true),
  ik_action_client_("r_arm_ik", true)
{
  ros::NodeHandle node_private("~");
  double shoulder_pan_,shoulder_lift_,upper_arm_roll_,elbow_flex_,forearm_roll_,wrist_flex_,wrist_roll_;
  node_private.getParam("pose_suggestion/shoulder_pan", shoulder_pan_ );
  node_private.getParam("pose_suggestion/shoulder_lift", shoulder_lift_ );
  node_private.getParam("pose_suggestion/upper_arm_roll", upper_arm_roll_);
  node_private.getParam("pose_suggestion/elbow_flex", elbow_flex_);
  node_private.getParam("pose_suggestion/forearm_roll", forearm_roll_);
  node_private.getParam("pose_suggestion/wrist_flex",  wrist_flex_);
  node_private.getParam("pose_suggestion/wrist_roll",  wrist_roll_);
    
  ik_goal_.ik_seed.name.push_back("r_shoulder_pan_joint");
  ik_goal_.ik_seed.name.push_back("r_shoulder_lift_joint");
  ik_goal_.ik_seed.name.push_back("r_upper_arm_roll_joint");
  ik_goal_.ik_seed.name.push_back("r_elbow_flex_joint");
  ik_goal_.ik_seed.name.push_back("r_forearm_roll_joint");
  ik_goal_.ik_seed.name.push_back("r_wrist_flex_joint");
  ik_goal_.ik_seed.name.push_back("r_wrist_roll_joint");
  ik_goal_.ik_seed.position.push_back(shoulder_pan_);
  ik_goal_.ik_seed.position.push_back(shoulder_lift_);
  ik_goal_.ik_seed.position.push_back(upper_arm_roll_);
  ik_goal_.ik_seed.position.push_back(elbow_flex_);
  ik_goal_.ik_seed.position.push_back(forearm_roll_);
  ik_goal_.ik_seed.position.push_back(wrist_flex_);
  ik_goal_.ik_seed.position.push_back(wrist_roll_);
};


GraspHandleAction::~GraspHandleAction()
{};



void GraspHandleAction::execute(const door_msgs::DoorGoalConstPtr& goal)
{
  ROS_INFO("GraspHandleAction: execute");

  // transform door message to time fixed frame
  door_msgs::Door goal_tr;
  if (!transformTo(tf_, fixed_frame, goal->door, goal_tr, fixed_frame)){
    ROS_ERROR("GraspHandleAction: Could not tranform door message from '%s' to '%s' at time %f",
	      goal->door.header.frame_id.c_str(), fixed_frame.c_str(), goal->door.header.stamp.toSec());
    action_server_.setAborted();
    return;
  }

  Vector x_axis(1,0,0);
  Vector normal = getDoorNormal(goal_tr);
  Vector wrist_pos_approach = normal*-0.25; // gripper 5 cm away from door handle
  Vector wrist_pos_grasp = normal*-0.15; // gripper 5 cm over the door handle
  Vector handle(goal_tr.handle.x, goal_tr.handle.y, goal_tr.handle.z);
  Stamped<Pose> gripper_pose;
  gripper_pose.frame_id_ = fixed_frame;
  
  // check for preemption
  if (action_server_.isPreemptRequested()){
    ROS_ERROR("GraspHandleAction: preempted");
    action_server_.setPreempted();
    return;
  }
  
  // open the gripper 
  pr2_controllers_msgs::Pr2GripperCommandGoal gripper_msg;
  gripper_msg.command.position = 0.07;
  gripper_msg.command.max_effort = gripper_effort;

  int MAX_OPEN_GRIPPER_RETRIES = 5;
  int open_gripper_retry = 0;
  while (gripper_action_client_.sendGoalAndWait(gripper_msg, ros::Duration(20.0), ros::Duration(5.0)) != SimpleClientGoalState::SUCCEEDED){

    if (open_gripper_retry >= MAX_OPEN_GRIPPER_RETRIES) {
      ROS_ERROR("GraspHandleAction: OPEN DOOR DEMO FAILED due to GraspHandleAction failure: gripper failed to open");
      action_server_.setAborted();
      return;
    }

    open_gripper_retry++;
    ROS_INFO("Failed to open gripper to %fm, retry attempt #%d",gripper_msg.command.position,open_gripper_retry);

  }


  // move gripper in front of door
  gripper_pose.setOrigin( Vector3(handle(0) + wrist_pos_approach(0), handle(1) + wrist_pos_approach(1),handle(2) + wrist_pos_approach(2)));
  gripper_pose.setRotation( tf::createQuaternionFromRPY(M_PI/2.0, 0, getVectorAngle(x_axis, normal)) ); 
  gripper_pose.stamp_ = Time::now();
  poseStampedTFToMsg(gripper_pose, ik_goal_.pose);
  ROS_INFO("GraspHandleAction: move in front of handle");
  if (ik_action_client_.sendGoalAndWait(ik_goal_, ros::Duration(20.0), ros::Duration(5.0)) != SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("GraspHandleAction: failed to move gripper in front of handle");
    ros::Duration(5.0).sleep();
    gripper_pose.stamp_ = Time::now();
    poseStampedTFToMsg(gripper_pose, ik_goal_.pose);
    //action_server_.setAborted();
    //return;
  }

  // move gripper over door handle
  gripper_pose.frame_id_ = fixed_frame;
  gripper_pose.setOrigin( Vector3(handle(0) + wrist_pos_grasp(0), handle(1) + wrist_pos_grasp(1),handle(2) + wrist_pos_grasp(2)));
  gripper_pose.stamp_ = Time::now();
  poseStampedTFToMsg(gripper_pose, ik_goal_.pose);
  ROS_INFO("GraspHandleAction: move over handle");
  if (ik_action_client_.sendGoalAndWait(ik_goal_, ros::Duration(20.0), ros::Duration(5.0)) != SimpleClientGoalState::SUCCEEDED) {  
    ROS_ERROR("Failure is not a problem here.");
    // do not error because we're touching the door
  }
  
  // close the gripper during 4 seconds
  gripper_msg.command.position = 0.0;
  if (gripper_action_client_.sendGoalAndWait(gripper_msg, ros::Duration(20.0), ros::Duration(5.0)) != SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("GraspHandleAction: gripper failed to close all the way, might be holding the handle");
    if (gripper_action_client_.getResult()->stalled)
    {
      ROS_INFO("GraspHandleAction: gripper failed to close all the way but is stalled, possibly holding a handle?");
    }
    else
    {
      ROS_INFO("GraspHandleAction: gripper failed to close all the way and is not stalled");
      action_server_.setAborted();
      return;
    }
  }

  action_result_.door = goal_tr;
  action_server_.setSucceeded(action_result_);  
}


