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
#include <geometry_msgs/PoseStamped.h>
#include "pr2_doors_actions/action_push_door.h"

using namespace tf;
using namespace KDL;
using namespace ros;
using namespace std;
using namespace door_handle_detector;
using namespace pr2_doors_common;
using namespace actionlib;

static const string fixed_frame = "odom_combined";
static const double push_dist = 0.65;
static const double push_vel  = 10.0 * M_PI/180.0;  // 10 [deg/sec]



PushDoorAction::PushDoorAction(tf::TransformListener& tf) : 
  tf_(tf),
  action_server_(ros::NodeHandle(), 
		 "push_door", 
		 boost::bind(&PushDoorAction::execute, this, _1))
{
  pose_pub_ = node_.advertise<geometry_msgs::PoseStamped>("r_arm_constraint_cartesian_pose_controller/command",20);
};


PushDoorAction::~PushDoorAction()
{};



void PushDoorAction::execute(const door_msgs::DoorGoalConstPtr& goal)
{
  ROS_INFO("PushDoorAction: execute");
 
  // transform door message to time fixed frame
  door_msgs::Door goal_tr;
  if (!transformTo(tf_, fixed_frame, goal->door, goal_tr, fixed_frame)){
    ROS_ERROR("Could not tranform door message from '%s' to '%s' at time %f",
	      goal->door.header.frame_id.c_str(), fixed_frame.c_str(), goal->door.header.stamp.toSec());
    action_server_.setPreempted();
    return;
  }

  // start monitoring gripper pose
  pose_state_received_ = false;
  Subscriber sub = node_.subscribe("r_arm_constraint_cartesian_pose_controller/state/pose", 1,  &PushDoorAction::poseCallback, this);
  Duration timeout = Duration().fromSec(3.0);
  Duration poll = Duration().fromSec(0.1);
  Time start_time = ros::Time::now();
  while (!pose_state_received_){
    if (start_time + timeout < ros::Time::now()){
      ROS_ERROR("failed to receive pose state");
      action_server_.setPreempted();
      return;
    }
    poll.sleep();
  }

  // angle step
  Duration sleep_time(0.01);
  double angle_step = 0;
  StampedTransform shoulder_pose; tf_.lookupTransform(goal_tr.header.frame_id, "r_shoulder_pan_link", Time(), shoulder_pose);
  if (goal_tr.rot_dir == door_msgs::Door::ROT_DIR_CLOCKWISE)
    angle_step = -push_vel*sleep_time.toSec();
  else if (goal_tr.rot_dir == door_msgs::Door::ROT_DIR_COUNTERCLOCKWISE)
    angle_step = push_vel*sleep_time.toSec();
  else{
    ROS_ERROR("door rotation direction not specified");
    action_server_.setPreempted();
    return;
  }

  // push door
  Stamped<Pose> gripper_pose;
  geometry_msgs::PoseStamped gripper_pose_msg;
  double angle = getNearestDoorAngle(shoulder_pose, goal_tr, 0.75, push_dist);
  while (!action_server_.isPreemptRequested()){
    sleep_time.sleep();

    // define griper pose
    gripper_pose = getGripperPose(goal_tr, angle, push_dist);
    gripper_pose.stamp_ = Time::now();
    poseStampedTFToMsg(gripper_pose, gripper_pose_msg);
    pose_pub_.publish(gripper_pose_msg);

    // increase angle when pose error is small enough
    boost::mutex::scoped_lock lock(pose_mutex_);
    if (fabs(angle) < M_PI/2.0)// && (gripper_pose.getOrigin() - pose_state_.getOrigin()).length() < 0.5)
      angle += angle_step;
  }
  ROS_ERROR("PushDoorAction: preempted");
  action_server_.setPreempted();
}


void PushDoorAction::poseCallback(const PoseConstPtr& pose)
{
  boost::mutex::scoped_lock lock(pose_mutex_);
  poseStampedMsgToTF(*pose, pose_state_);
  pose_state_received_ = true;
}

