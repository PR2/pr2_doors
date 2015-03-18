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

#include "pr2_doors_common/door_functions.h"
#include "pr2_doors_actions/action_open_door.h"


using namespace tf;
using namespace KDL;
using namespace ros;
using namespace std;
using namespace door_handle_detector;
using namespace pr2_doors_common;
using namespace actionlib;

static const string fixed_frame = "odom_combined";


OpenDoorAction::OpenDoorAction(tf::TransformListener& tf) :
  tf_(tf),
  action_server_(ros::NodeHandle(), 
		 "open_door", 
		 boost::bind(&OpenDoorAction::execute, this, _1))
{
  ros::NodeHandle node;
  tff_pub_ = node.advertise<tff_controller::TaskFrameFormalism>("r_arm_cartesian_tff_controller/command", 10);
};



OpenDoorAction::~OpenDoorAction()
{};



void OpenDoorAction::execute(const door_msgs::DoorGoalConstPtr& goal)
{ 
  ROS_INFO("OpenDoorAction: execute");

  // open door
  tff_door_.mode.linear.x = tff_door_.VELOCITY;
  tff_door_.mode.linear.y = tff_door_.FORCE;
  tff_door_.mode.linear.z = tff_door_.FORCE;
  tff_door_.mode.angular.x = tff_door_.FORCE;
  tff_door_.mode.angular.y = tff_door_.FORCE;
  tff_door_.mode.angular.z = tff_door_.POSITION;
  
  tff_door_.value.linear.x = getDoorDir(goal->door) * 0.45;
  tff_door_.value.linear.y = 0.0;
  tff_door_.value.linear.z = 0.0;
  tff_door_.value.angular.x = 0.0;
  tff_door_.value.angular.y = 0.0;
  tff_door_.value.angular.z = 0.0;
  
  // open door
  while (ros::ok() && !action_server_.isPreemptRequested()){
    tff_pub_.publish(tff_door_);
    Duration(0.1).sleep();
  }
  
  // preempted
  ROS_INFO("ActionOpenDoor: Preempted");
  
  action_server_.setPreempted();
}

