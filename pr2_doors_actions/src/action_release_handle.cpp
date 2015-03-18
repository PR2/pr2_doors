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

#include "pr2_doors_actions/action_release_handle.h"
#include <tf/tf.h>

using namespace tf;
using namespace KDL;
using namespace ros;
using namespace std;
using namespace door_handle_detector;
using namespace actionlib;

static const string fixed_frame = "odom_combined";


ReleaseHandleAction::ReleaseHandleAction(tf::TransformListener& tf) : 
  tf_(tf),
  action_server_(ros::NodeHandle(), 
		 "release_handle", 
		 boost::bind(&ReleaseHandleAction::execute, this, _1)),
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


ReleaseHandleAction::~ReleaseHandleAction()
{};



void ReleaseHandleAction::execute(const door_msgs::DoorGoalConstPtr& goal)
{
  ROS_INFO("ReleaseHandleAction: execute");

  // open the gripper
  ROS_INFO("ReleaseHandleAction: open the gripper");
  pr2_controllers_msgs::Pr2GripperCommandGoal gripper_msg;
  gripper_msg.command.position = 0.07; //full open
  gripper_msg.command.max_effort = 10000.0;


  int MAX_OPEN_GRIPPER_RETRIES = 5;
  int open_gripper_retry = 0;
  while (gripper_action_client_.sendGoalAndWait(gripper_msg, ros::Duration(20.0), ros::Duration(5.0)) != SimpleClientGoalState::SUCCEEDED){

    if (open_gripper_retry >= MAX_OPEN_GRIPPER_RETRIES) {
      ROS_ERROR("Release handle: gripper failed to open");
      action_server_.setAborted();
      return;
    }

    open_gripper_retry++;
    ROS_INFO("Release handle: Failed to open gripper to %fm, retry attempt #%d",gripper_msg.command.position,open_gripper_retry);

  }

  // get current gripper pose
  ros::Time time = ros::Time::now();
  if (!tf_.waitForTransform("base_link", "r_wrist_roll_link", time, ros::Duration(3.0))){
    ROS_ERROR("Release handle: Failed to get transform between base_link and r_wrist_roll_link");
    action_server_.setAborted();
    return;
  }
  tf::StampedTransform gripper_pose;
  tf_.lookupTransform("base_link", "r_wrist_roll_link", time, gripper_pose);
  tf::Transform gripper_offset(tf::Quaternion::getIdentity() ,tf::Vector3(-0.06,0,0));


  // move gripper away from the door
  ROS_INFO("Release handle: move away from door handle");
  tf::poseStampedTFToMsg(tf::Stamped<tf::Transform>(gripper_pose * gripper_offset, time, "base_link"),
                         ik_goal_.pose);
  if (ik_action_client_.sendGoalAndWait(ik_goal_, ros::Duration(20.0), ros::Duration(5.0)) != SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("Release handle: failed to move away from door handle");
    action_server_.setAborted();
    return;
  }

  action_result_.door = goal->door;
  action_server_.setSucceeded(action_result_);  
}



