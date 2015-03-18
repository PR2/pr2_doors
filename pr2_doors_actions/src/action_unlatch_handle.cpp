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

#include "pr2_doors_actions/action_unlatch_handle.h"
#include "pr2_doors_common/door_functions.h"


using namespace tf;
using namespace KDL;
using namespace ros;
using namespace std;
using namespace door_handle_detector;
using namespace pr2_doors_common;
using namespace actionlib;

static const string fixed_frame = "odom_combined";


UnlatchHandleAction::UnlatchHandleAction(tf::TransformListener& tf) :
  tf_(tf),
  action_server_(ros::NodeHandle(), 
		 "unlatch_handle", 
		 boost::bind(&UnlatchHandleAction::execute, this, _1))
{
  tff_pub_ = node_.advertise<tff_controller::TaskFrameFormalism>("r_arm_cartesian_tff_controller/command", 10);
};



UnlatchHandleAction::~UnlatchHandleAction()
{};



void UnlatchHandleAction::execute(const  door_msgs::DoorGoalConstPtr& goal)
{ 
  ROS_INFO("UnlatchHandleAction: execute");

  // push/pull dir
  double door_dir = getDoorDir(goal->door);
  double handle_dir = getHandleDir(goal->door);

  // stop
  tff_stop_.mode.linear.x = tff_stop_.FORCE;
  tff_stop_.mode.linear.y = tff_stop_.FORCE;
  tff_stop_.mode.linear.z = tff_stop_.FORCE;
  tff_stop_.mode.angular.x = tff_stop_.FORCE;
  tff_stop_.mode.angular.y = tff_stop_.FORCE;
  tff_stop_.mode.angular.z = tff_stop_.FORCE;
  
  tff_stop_.value.linear.x = 0.0;
  tff_stop_.value.linear.y = 0.0;
  tff_stop_.value.linear.z = 0.0;
  tff_stop_.value.angular.x = 0.0;
  tff_stop_.value.angular.y = 0.0;
  tff_stop_.value.angular.z = 0.0;
  
  // turn handle
  tff_handle_.mode.linear.x = tff_handle_.FORCE;
  tff_handle_.mode.linear.y = tff_handle_.FORCE;
  tff_handle_.mode.linear.z = tff_handle_.FORCE;
  tff_handle_.mode.angular.x = tff_handle_.FORCE;
  tff_handle_.mode.angular.y = tff_handle_.FORCE;
  tff_handle_.mode.angular.z = tff_handle_.POSITION;
  
  tff_handle_.value.linear.x = 20.0 * door_dir;
  tff_handle_.value.linear.y = 0.0;
  tff_handle_.value.linear.z = 0.0;
  tff_handle_.value.angular.x = 0.0;
  tff_handle_.value.angular.y = 0.0;
  tff_handle_.value.angular.z = 0.0;
  

  // start monitoring tf position
  Subscriber sub = node_.subscribe("r_arm_cartesian_tff_controller/state/position", 1,  &UnlatchHandleAction::tffCallback, this);
  Duration timeout = Duration().fromSec(3.0);
  Duration poll = Duration().fromSec(0.1);
  Time start_time = ros::Time::now();
  tff_state_received_ = false;
  ROS_INFO("UnlatchHandleAction: waiting to receive tff state");
  while (!tff_state_received_){
    if (start_time + timeout < ros::Time::now()){
      ROS_ERROR("UnlatchHandleAction: failed to receive tff state");
      action_server_.setPreempted();
      return;
    }
    poll.sleep();
  }

  double sleep_time = 0.1;
  ros::Duration wait_after_door_moved = Duration().fromSec(1.0);
  ros::Time time_door_moved;
  action_result_.door = goal->door;

  // turn handle and push door, until door moves forward
  ROS_INFO("UnlatchHandleAction: start turning handle");
  tff_handle_.value.angular.x = handle_dir * 0.5;
  while (time_door_moved == ros::Time() || ros::Time::now() < time_door_moved + wait_after_door_moved){
    Duration(sleep_time).sleep();
    boost::mutex::scoped_lock lock(tff_mutex_);

    // increase torque to turn handle until door is opened 5 cm
    if (fabs(tff_state_.linear.x) < 0.05)
      tff_handle_.value.angular.x += handle_dir * 0.5 * sleep_time; // add 0.5 Nm per second
    else{
      tff_handle_.value.angular.x = 0;
      if (time_door_moved == Time())
	time_door_moved = ros::Time::now();
    }
    tff_pub_.publish(tff_handle_);

    // detect when gripper is not on hanlde
    if (fabs(tff_state_.angular.x) > M_PI/2.0 || fabs(tff_state_.linear.y) > 0.1 || fabs(tff_state_.linear.z) > 0.1){
      //fabs(tff_state_.angular.y) > M_PI/8.0 || fabs(tff_state_.angular.z) > M_PI/8.0){
      tff_pub_.publish(tff_stop_);
      lock.unlock();
      ROS_ERROR("UnlatchHandleAction: Gripper was not on door handle");
      action_server_.setPreempted();
      return;
    }

    // detect when door is locked
    if (fabs(tff_handle_.value.angular.x) > 3.5 && fabs(tff_state_.angular.x) < M_PI/6.0){
      tff_pub_.publish(tff_stop_);
      lock.unlock();
      ROS_INFO("UnlatchHandleAction: Door is locked");
      action_result_.door.latch_state = door_msgs::Door::LOCKED;
      action_server_.setSucceeded(action_result_);  
      return;
    }

    // check for preemption
    if (action_server_.isPreemptRequested()){
      ROS_ERROR("UnlatchHandleAction: preempted");
      action_server_.setPreempted();
      return;
    }
  }
  
  // finished
  action_result_.door.latch_state = door_msgs::Door::UNLATCHED;
  action_server_.setSucceeded(action_result_);  
}


void UnlatchHandleAction::tffCallback(const TffConstPtr& tff)
{
  boost::mutex::scoped_lock lock(tff_mutex_);
  tff_state_ = *tff;
  tff_state_received_ = true;
}

