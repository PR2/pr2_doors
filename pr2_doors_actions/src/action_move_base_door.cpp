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
#include "pr2_doors_actions/action_move_base_door.h"


using namespace tf;
using namespace KDL;
using namespace ros;
using namespace std;
using namespace door_handle_detector;
using namespace pr2_doors_common;
using namespace actionlib;

static const string fixed_frame = "odom_combined";
static const double motion_step = 0.01; // 1 cm steps
static const double update_rate = 10.0; // 10 hz

MoveBaseDoorAction::MoveBaseDoorAction(tf::TransformListener& tf) :
  tf_(tf),
  costmap_ros_("costmap_move_base_door", tf),
  costmap_model_(costmap_),
  action_server_(ros::NodeHandle(), 
		 "move_base_door", 
		 boost::bind(&MoveBaseDoorAction::execute, this, _1))
{
  costmap_ros_.stop();

  ros::NodeHandle node;
  base_pub_ = node.advertise<geometry_msgs::Twist>("base_controller/command", 10);

  search_pattern_sideways_.push_back(0.0);
  search_pattern_sideways_.push_back(-1.0);
  search_pattern_sideways_.push_back(1.0);
  search_pattern_sideways_.push_back(-1.0);
  search_pattern_sideways_.push_back(1.0);

  search_pattern_forward_.push_back(1.0);
  search_pattern_forward_.push_back(1.0);
  search_pattern_forward_.push_back(1.0);
  search_pattern_forward_.push_back(0.0);
  search_pattern_forward_.push_back(0.0);

  footprint_ = costmap_ros_.getRobotFootprint();
};



MoveBaseDoorAction::~MoveBaseDoorAction()
{};



geometry_msgs::Point MoveBaseDoorAction::toPoint(const tf::Vector3& pnt)
{
  geometry_msgs::Point res;
  res.x = pnt.x();
  res.y = pnt.y();
  res.z = pnt.z();
  return res;
}

geometry_msgs::Vector3 MoveBaseDoorAction::toVector(const tf::Vector3& pnt)
{
  geometry_msgs::Vector3 res;
  res.x = pnt.x();
  res.y = pnt.y();
  res.z = pnt.z();
  return res;
}



std::vector<geometry_msgs::Point> MoveBaseDoorAction::getOrientedFootprint(const tf::Vector3 pos, double theta_cost)
{
  double cos_th = cos(theta_cost);
  double sin_th = sin(theta_cost);
  std::vector<geometry_msgs::Point> oriented_footprint;
  for(unsigned int i = 0; i < footprint_.size(); ++i){
    geometry_msgs::Point new_pt;
    new_pt.x = pos.x() + (footprint_[i].x * cos_th - footprint_[i].y * sin_th);
    new_pt.y = pos.y() + (footprint_[i].x * sin_th + footprint_[i].y * cos_th);
    oriented_footprint.push_back(new_pt);
  }
  return oriented_footprint;
}



void MoveBaseDoorAction::execute(const door_msgs::DoorGoalConstPtr& goal)
{ 
  ROS_INFO("MoveBaseDoorAction: execute");

  costmap_ros_.start();
  ros::Duration(3.0).sleep();

  // get path from current pose to goal
  door_msgs::Door door;
  if (!tf_.waitForTransform(fixed_frame, goal->door.header.frame_id, goal->door.header.stamp, ros::Duration(3.0)) ||
      !transformTo(tf_, fixed_frame, goal->door, door, fixed_frame)){
    ROS_ERROR("MoveBaseDoorAction: could not transform from %s to %s", fixed_frame.c_str(), goal->door.header.frame_id.c_str());
    action_server_.setAborted();
    return;
  }
  tf::Pose end_position = getRobotPose(door, 0.5);
  tf::Stamped<tf::Pose> start_position;
  costmap_ros_.getRobotPose(start_position);
  ROS_DEBUG("MoveBaseDoorAction: current robot pose is %f %f %f", start_position.getOrigin().x(), start_position.getOrigin().y(), start_position.getOrigin().z());
  ROS_DEBUG("MoveBaseDoorAction: goal robot pose is %f %f %f", end_position.getOrigin().x(), end_position.getOrigin().y(), end_position.getOrigin().z());

  // get motion and search direction in fixed frame
  tf::Vector3 motion_direction = (end_position.getOrigin() - start_position.getOrigin()).normalize();
  tf::Vector3 search_direction = motion_direction.cross(tf::Vector3(0,0,1)).normalize();
  ROS_DEBUG("MoveBaseDoorAction: motion direction: %f %f %f", motion_direction.x(), motion_direction.y(), motion_direction.z());
  ROS_DEBUG("MoveBaseDoorAction: search direction: %f %f %f", search_direction.x(), search_direction.y(), search_direction.z());

  ros::Rate rate(update_rate);  
  while (ros::ok() && !action_server_.isPreemptRequested()){
    // copy most recent costmap into costmap model
    costmap_ros_.getCostmapCopy(costmap_);

    // get current robot pose
    tf::Stamped<tf::Pose> current_pose;
    costmap_ros_.getRobotPose(current_pose);
    tf::Vector3 current_position = current_pose.getOrigin();
    double current_orientation = tf::getYaw(current_pose.getRotation());

    // check if we reached our goal
    ROS_DEBUG("MoveBaseDoorAction: distance to goal: %f", motion_direction.dot(end_position.getOrigin() - current_position));
    if (motion_direction.dot(end_position.getOrigin() - current_position) < 0){
      geometry_msgs::Twist base_twist;
      base_pub_.publish(base_twist);  // stop moving
      door_msgs::DoorResult result;  result.door = door;
      action_server_.setSucceeded(result);
      costmap_ros_.stop();
      ROS_INFO("MoveBaseDoorAction: reached goal");
      return;
    }

    // find next valid robot pose
    tf::Vector3 next_position;
    bool success = false;
    for (int i=0; i<(int)search_pattern_sideways_.size(); i++){
      next_position = current_position +  (motion_direction*motion_step*search_pattern_forward_[i])  + (search_direction * motion_step * search_pattern_sideways_[i]);

      // check in costmap if this is valid robot pose
      if (costmap_model_.footprintCost(toPoint(next_position), 
				       getOrientedFootprint(next_position, current_orientation), 
				       costmap_ros_.getInscribedRadius(), 
				       costmap_ros_.getCircumscribedRadius()) >= 0){
	ROS_DEBUG("MoveBaseDoorAction: Valid robot position: %f %f %f", next_position.x(), next_position.y(), next_position.z());
	success = true;
	break;
      }
      ROS_DEBUG("MoveBaseDoorAction: IN-Valid robot position: %f %f %f", next_position.x(), next_position.y(), next_position.z());
    }
    geometry_msgs::Twist base_twist;
    if (success){
      base_twist.linear = toVector(((current_pose.inverse() * tf::Pose(tf::Quaternion::getIdentity(), next_position)).getOrigin()) * update_rate );
      // base_twist.linear = toVector((next_position - current_position)*update_rate);
    }
    //ROS_DEBUG("MoveBaseDoorAction: Commanding base: %f %f == %f", base_twist.linear.x, base_twist.linear.y, base_twist.angular.z);
    base_pub_.publish(base_twist);

    rate.sleep();
  }

  costmap_ros_.stop();
  action_server_.setPreempted();
}

