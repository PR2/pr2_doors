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

#include <door_handle_detector/DoorsDetectorCloud.h>
#include <door_handle_detector/geometric_helper.h>
#include <pr2_doors_common/door_functions.h>
#include <pr2_laser_snapshotter/BuildCloudAngle.h>
//#include <pr2_robot_actions/set_hokuyo_mode.h>
#include "pr2_doors_actions/action_detect_door.h"
#include <pr2_laser_snapshotter/TiltLaserSnapshotAction.h>
#include <actionlib/client/simple_action_client.h>


using namespace ros;
using namespace std;
using namespace door_handle_detector;
using namespace pr2_doors_common;
using namespace actionlib;

static const string fixed_frame = "odom_combined";




DetectDoorAction::DetectDoorAction(tf::TransformListener& tf):
  tf_(tf),
  action_server_(ros::NodeHandle(), 
		 "detect_door", 
		 boost::bind(&DetectDoorAction::execute, this, _1)),
  laserSnapshotActionClient_("point_cloud_action/single_sweep_cloud")
{};


DetectDoorAction::~DetectDoorAction(){};


void DetectDoorAction::execute(const door_msgs::DoorGoalConstPtr& goal)
{ 
  ROS_INFO("DetectDoorAction: execute");

  // transform door message to time fixed frame
  door_msgs::Door goal_tr;
  if (!transformTo(tf_, fixed_frame, goal->door, goal_tr, fixed_frame)){
    ROS_ERROR("DetectDoorAction: Could not tranform door message from '%s' to '%s' at time %f",
	      goal->door.header.frame_id.c_str(), fixed_frame.c_str(), goal->door.header.stamp.toSec());
    action_server_.setPreempted();
    return;
  }
  ROS_INFO("DetectDoorAction: goal message transformed to frame %s", fixed_frame.c_str());
  door_msgs::Door result_laser;
  if (!laserDetection(goal_tr, result_laser)){
    ROS_ERROR("DetectDoorAction: Aborted");
    action_server_.setPreempted();
    return;
  }

  if (getDoorDir(result_laser) < 0){
    ROS_INFO("Detected door that opens towards robot");
    action_server_.setPreempted();
    return;
  }

  ROS_INFO("DetectDoorAction: Succeeded");
  action_result_.door = result_laser;
  action_server_.setSucceeded(action_result_);  
}

bool DetectDoorAction::laserDetection(const door_msgs::Door& door_in, door_msgs::Door& door_out)
{
  // check where robot is relative to door
  if (action_server_.isPreemptRequested()) return false;

  if (!tf_.waitForTransform("base_footprint", "laser_tilt_link", ros::Time(), ros::Duration().fromSec(5.0))){
    ROS_ERROR("DetectDoorAction: error getting transform from 'base_footprint' to 'laser_tilt_link'");
    return false;
  }
  tf::StampedTransform tilt_stage;
  tf_.lookupTransform("base_footprint", "laser_tilt_link", ros::Time(), tilt_stage);
  double laser_height = tilt_stage.getOrigin()[2];
  tf::Stamped<tf::Vector3> doorpoint(tf::Vector3((door_in.frame_p1.x+door_in.frame_p2.x)/2.0,
						 (door_in.frame_p1.y+door_in.frame_p2.y)/2.0,
						 (door_in.frame_p1.z+door_in.frame_p2.z)/2.0),
				     ros::Time(), door_in.header.frame_id);

  if (action_server_.isPreemptRequested()) return false;

  if (!tf_.waitForTransform("base_footprint", doorpoint.frame_id_, ros::Time(), ros::Duration().fromSec(5.0))){
    ROS_ERROR("DetectDoorAction: error getting transform from '%s' to 'base_footprint'", doorpoint.frame_id_.c_str());
    return false;
  }

  tf_.transformPoint("base_footprint", doorpoint, doorpoint);
  doorpoint[2] = 0;
  double dist = doorpoint.length();
  double door_bottom = -0.5;
  double door_top    =  2.5;
  ROS_INFO("DetectDoorAction: tilt laser is at height %f, and door at distance %f", laser_height, dist);

  // set the laser scanner to NON-intensity mode
  NodeHandle node;
  bool use_sim_time;
  node.param("use_sim_time", use_sim_time, false);
  if (!use_sim_time){
    ROS_WARN("dynamic_reconfigure of tilt_hokuyo_node is commented out, awaiting #3894 from simulation");
    system("rosrun dynamic_reconfigure dynparam set /tilt_hokuyo_node '{ intensity: false }'");
    system("rosrun dynamic_reconfigure dynparam set /tilt_hokuyo_node '{ skip: 0 }'");
  }

  // gets a point cloud from the point_cloud_srv
  if (action_server_.isPreemptRequested()) return false;

  ROS_INFO("DetectDoorAction: get a point cloud from the door");
  pr2_laser_snapshotter::TiltLaserSnapshotGoal goalMsg;

  goalMsg.angle_begin = -atan2(door_top - laser_height, dist);
  goalMsg.angle_end = atan2(laser_height - door_bottom, dist);
  goalMsg.duration = 10.0;


  laserSnapshotActionClient_.waitForServer();
  laserSnapshotActionClient_.sendGoal(goalMsg);
  laserSnapshotActionClient_.waitForResult();

  if (laserSnapshotActionClient_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_ERROR("DetectDoorAction: failed to get point cloud for door detection");
    return false;
  }
  pr2_laser_snapshotter::TiltLaserSnapshotResultConstPtr res_pointcloud = laserSnapshotActionClient_.getResult();

  // detect door
  if (action_server_.isPreemptRequested()) return false;

  ROS_INFO("DetectDoorAction: detect the door in a pointcloud of size %u", (unsigned int)res_pointcloud->cloud.points.size());
  door_handle_detector::DoorsDetectorCloud::Request  req_doordetect;
  door_handle_detector::DoorsDetectorCloud::Response res_doordetect;
  req_doordetect.door = door_in;
  req_doordetect.cloud = res_pointcloud->cloud;

  if (!ros::service::call("doors_detector_cloud", req_doordetect, res_doordetect)){
    ROS_ERROR("DetectDoorAction: failed to detect a door");
    return false;
  }

  door_out = res_doordetect.doors[0];
  return true;
}

