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
#include <door_handle_detector/DoorsDetector.h>
#include <pr2_doors_common/door_functions.h>
#include <door_msgs/Door.h>
#include <boost/thread/thread.hpp>
#include <pr2_laser_snapshotter/BuildCloudAngle.h>
#include <kdl/frames.hpp>
//#include <pr2_robot_actions/set_hokuyo_mode.h>
#include <pr2_controllers_msgs/PointHeadActionGoal.h>
#include "pr2_doors_actions/action_detect_handle.h"


using namespace ros;
using namespace std;
using namespace door_handle_detector;
using namespace pr2_doors_common;
using namespace KDL;
using namespace actionlib;

static const string fixed_frame = "odom_combined";
static const double scan_speed  = 0.1; // [m/sec]
static const double scan_height = 0.4; //[m]
static const unsigned int max_retries = 5;
static const double handle_dimension = 0.07; // [m] this is the radius of a half circle approximating the handle

DetectHandleAction::DetectHandleAction(tf::TransformListener& tf):
  tf_(tf),
  action_server_(ros::NodeHandle(), 
		 "detect_handle", 
		 boost::bind(&DetectHandleAction::execute, this, _1)),
  laserSnapshotActionClient_("point_cloud_action/single_sweep_cloud")
{
  NodeHandle node;
  pub_ = node.advertise<pr2_controllers_msgs::PointHeadActionGoal>("head_traj_controller/point_head_action/goal",10);
};


DetectHandleAction::~DetectHandleAction()
{
};



void DetectHandleAction::execute(const door_msgs::DoorGoalConstPtr& goal)
{
  ROS_INFO("DetectHandleAction: execute");

  // transform door message to time fixed frame
  door_msgs::Door goal_tr;
  transformTo(tf_, fixed_frame, goal->door, goal_tr, fixed_frame);

  // try to detect handle
  for (unsigned int nr_tries=0; nr_tries<max_retries; nr_tries++){

    // check for preemption
    if (action_server_.isPreemptRequested()){
      ROS_INFO("DetectHandleAction: Preempted");
      action_server_.setPreempted();
      return;
    }

    boost::thread* thread_laser;//,* thread_camera;
    bool success_laser, success_camera;
    door_msgs::Door result_laser, result_camera;
    thread_laser =  new boost::thread(boost::bind(&DetectHandleAction::laserDetectionFunction, this, goal_tr, &result_laser, &success_laser));
    //thread_camera = new boost::thread(boost::bind(&DetectHandleAction::cameraDetectionFunction, this, goal_tr, &result_camera, &success_camera));
    thread_laser->join();
    //thread_camera->join();
    delete thread_laser;
    //delete thread_camera;
    result_camera = result_laser;
    success_camera = success_laser;

    ROS_INFO("Laser success %i, Camera success %i", success_laser, success_camera);

    if (!success_laser && !success_camera){
      ROS_ERROR("Laser and Camera detection failed");
      continue;
    }
    if (!success_laser){
      ROS_ERROR("Laser detection failed");
      continue;
    }
    if (!success_camera){
      ROS_ERROR("Camera detection failed");
      continue;
    }

    // transform laser data
    if (!transformTo(tf_, fixed_frame, result_laser, result_laser, fixed_frame)){
      ROS_ERROR ("Could not transform laser door message from frame %s to frame %s.",
		 result_laser.header.frame_id.c_str (), fixed_frame.c_str ());
      action_server_.setAborted();
      return;
    }
    ROS_INFO("detected handle position transformed to '%s'", fixed_frame.c_str());

    // transform camera data
    if (!transformTo(tf_, fixed_frame, result_camera, result_camera, fixed_frame)){
      ROS_ERROR ("Could not transform camera door message from frame %s to frame %s.",
		 result_camera.header.frame_id.c_str (), fixed_frame.c_str ());
      action_server_.setAborted();
      return;
    }
    ROS_INFO("detected handle position transformed to '%s'", fixed_frame.c_str());

    // compare laser and camera results
    double  error = sqrt(pow(result_laser.handle.x - result_camera.handle.x,2) +
			 pow(result_laser.handle.y - result_camera.handle.y,2) +
			 pow(result_laser.handle.z - result_camera.handle.z,2));
    ROS_INFO("difference between laser and camera result = %f", error);
    if (error < this->handle_laser_camera_distance_tol){
      // store handle position
      action_result_.door = result_laser;
      action_result_.door.handle.x = (result_laser.handle.x + result_camera.handle.x)/2.0;
      action_result_.door.handle.y = (result_laser.handle.y + result_camera.handle.y)/2.0;
      action_result_.door.handle.z = (result_laser.handle.z + result_camera.handle.z)/2.0;

      // take detection angle into account
      geometry_msgs::Point32 handle_robot_point;
      if (!transformPointTo(tf_, action_result_.door.header.frame_id, "base_footprint", action_result_.door.header.stamp,  action_result_.door.handle, handle_robot_point, fixed_frame, action_result_.door.header.stamp)){
	ROS_ERROR ("Could not transform handle from frame %s to frame %s.",
		   action_result_.door.header.frame_id.c_str (), string("base_footprint").c_str ());
	action_server_.setAborted();
	return;
      }
      geometry_msgs::Vector3 handle_robot_vec;
      handle_robot_vec.x = handle_robot_point.x;
      handle_robot_vec.y = handle_robot_point.y;
      handle_robot_vec.z = handle_robot_point.z;
      if (!transformVectorTo(tf_,"base_footprint", action_result_.door.header.frame_id, action_result_.door.header.stamp,  handle_robot_vec, handle_robot_vec, fixed_frame, action_result_.door.header.stamp)){
	ROS_ERROR ("Could not transform handle from frame %s to frame %s.",
		   action_result_.door.header.frame_id.c_str (), string("base_footprint").c_str ());
	action_server_.setAborted();
	return;
      }
      Vector handle_door(handle_robot_vec.x, handle_robot_vec.y, 0.0);
      handle_door.Normalize();
      Vector door_normal = getDoorNormal(action_result_.door);
      Vector handle_door_old = - handle_door * handle_dimension;
      Vector handle_door_new = Rotation::RotZ(getVectorAngle(handle_door, door_normal)) * handle_door_old;
      Vector handle(action_result_.door.handle.x, action_result_.door.handle.y, action_result_.door.handle.z);
      handle = handle - handle_door_old + handle_door_new;
      action_result_.door.handle.x = handle.x();
      action_result_.door.handle.y = handle.y();
      action_result_.door.handle.z = handle.z();

      ROS_INFO("Found handle in %i tries", nr_tries+1);
      action_result_.door = result_laser;
      action_server_.setSucceeded(action_result_);  
      return;
    }
    else
    {
      ROS_WARN("Distance between laser and camera handle detection is more than %f m apart",this->handle_laser_camera_distance_tol);
      ROS_WARN("         laser results: x = %f y = %f z = %f ", result_laser.handle.x, result_laser.handle.y, result_laser.handle.z);
      ROS_WARN("        camera results: x = %f y = %f z = %f ", result_camera.handle.x, result_camera.handle.y, result_camera.handle.z);
      ROS_WARN("Retrying handle detection");
    }
  }
  ROS_ERROR("Did not find hanlde in %i tries", max_retries);
  action_server_.setAborted();
}



void DetectHandleAction::laserDetectionFunction(const door_msgs::Door& door_in,
						door_msgs::Door* door_out, bool* success)
{
  *success = laserDetection(door_in, *door_out);
}

bool DetectHandleAction::laserDetection(const door_msgs::Door& door_in,
                                        door_msgs::Door& door_out)
{
  // check where robot is relative to the door
  if (!tf_.waitForTransform("base_footprint", "laser_tilt_link", ros::Time(), ros::Duration().fromSec(5.0))){
    ROS_ERROR("could not get transform from 'base_footprint' to 'laser_tilt_link'");
    return false;
  }
  tf::StampedTransform tilt_stage;
  tf_.lookupTransform("base_footprint", "laser_tilt_link", ros::Time(), tilt_stage);
  ROS_INFO("handle activate");
  double laser_height = tilt_stage.getOrigin()[2];
  tf::Stamped<tf::Vector3> handlepoint(tf::Vector3((door_in.door_p1.x+door_in.door_p2.x)/2.0,
						   (door_in.door_p1.y+door_in.door_p2.y)/2.0,
						   0.9),
				       ros::Time(), door_in.header.frame_id);
  if (!tf_.waitForTransform("base_footprint", handlepoint.frame_id_, ros::Time(), ros::Duration().fromSec(5.0))){
    ROS_ERROR("could not get transform from 'base_footprint' to '%s'", handlepoint.frame_id_.c_str());
    return false;
  }
  tf_.transformPoint("base_footprint", handlepoint, handlepoint);
  double handle_bottom = handlepoint[2]-(scan_height/2.0);
  double handle_top = handlepoint[2]+(scan_height/2.0);
  handlepoint[2] = 0;
  double dist = handlepoint.length();
  ROS_INFO("tilt laser is at height %f, and door at distance %f", laser_height, dist);

  // set the laser scanner to intensity mode

  NodeHandle node;
  bool use_sim_time;
  node.param("use_sim_time", use_sim_time, false);
  if (!use_sim_time){
    ROS_WARN("dynamic_reconfigure of tilt_hokuyo_node is commented out, awaiting #3894 from simulation");
    system("rosrun dynamic_reconfigure dynparam set /tilt_hokuyo_node '{ intensity: true }'");
    system("rosrun dynamic_reconfigure dynparam set /tilt_hokuyo_node '{ skip: 1 }'");
  }

  // gets a point cloud from the point_cloud_srv
  if (action_server_.isPreemptRequested()) return false;

  ROS_INFO("get a point cloud from the door");
  pr2_laser_snapshotter::TiltLaserSnapshotGoal goalMsg;

  laserSnapshotActionClient_.waitForServer();

  goalMsg.angle_begin = -atan2(handle_top - laser_height, dist);
  goalMsg.angle_end =atan2(laser_height - handle_bottom, dist);
  goalMsg.duration = scan_height/scan_speed;

  laserSnapshotActionClient_.sendGoal(goalMsg);
  laserSnapshotActionClient_.waitForResult();

  if (laserSnapshotActionClient_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_ERROR("DetectHandleAction: failed to get point cloud for handle detection");
    return false;
  }
  pr2_laser_snapshotter::TiltLaserSnapshotResultConstPtr res_pointcloud = laserSnapshotActionClient_.getResult();

  // detect handle
  if (action_server_.isPreemptRequested()) return false;
  ROS_INFO("start detecting the handle using the laser, in a pointcloud of size %u", (unsigned int)res_pointcloud->cloud.points.size());
  door_handle_detector::DoorsDetectorCloud::Request  req_handledetect;
  door_handle_detector::DoorsDetectorCloud::Response res_handledetect;
  req_handledetect.door = door_in;
  req_handledetect.cloud = res_pointcloud->cloud;
  if (!ros::service::call("handle_detector_cloud", req_handledetect, res_handledetect)){
    ROS_ERROR("failed to detect a handle using the laser");
    return false;
  }

  door_out = res_handledetect.doors[0];
  return true;
}


void DetectHandleAction::cameraDetectionFunction(const door_msgs::Door& door_in,
						 door_msgs::Door* door_out, bool* success)
{
  *success = cameraDetection(door_in, *door_out);
}


bool DetectHandleAction::cameraDetection(const door_msgs::Door& door_in,
                                         door_msgs::Door& door_out)
{
  // make the head point towards the door
  ROS_INFO("point head towards door");
  pr2_controllers_msgs::PointHeadGoal head_goal;
  head_goal.pointing_axis.x = 1;
  head_goal.pointing_axis.y = 0;
  head_goal.pointing_axis.z = 0;
  head_goal.pointing_frame = "head_tilt_link";
  head_goal.min_duration = ros::Duration(0.0);
  head_goal.max_velocity = 10.0;

  geometry_msgs::PointStamped door_pnt;
  door_pnt.point.z = 0.9;
  door_pnt.header.frame_id = door_in.header.frame_id;
  if (door_in.hinge == door_in.HINGE_P1){
    door_pnt.point.x = 0.1*door_in.door_p1.x + 0.9*door_in.door_p2.x;
    door_pnt.point.y = 0.1*door_in.door_p1.y + 0.9*door_in.door_p2.y;
  }
  else if (door_in.hinge == door_in.HINGE_P2){
    door_pnt.point.x = 0.9*door_in.door_p1.x + 0.1*door_in.door_p2.x;
    door_pnt.point.y = 0.9*door_in.door_p1.y + 0.1*door_in.door_p2.y;
  }
  else{
    ROS_ERROR("Door hinge side is not specified");
    return false;
  }
  cout << "door_pnt.point " << door_in.header.frame_id << " "
       << door_pnt.point.x << " "
       << door_pnt.point.y << " "
       <<  door_pnt.point.z << endl;
  head_goal.target = door_pnt;

  pr2_controllers_msgs::PointHeadActionGoal head_goal_action;
  head_goal_action.goal = head_goal;
  pub_.publish(head_goal_action);
  ros::Duration().fromSec(2).sleep();

  // detect handle
  if (action_server_.isPreemptRequested()) return false;
  ROS_INFO("start detecting the handle using the camera");
  door_handle_detector::DoorsDetector::Request  req_handledetect;
  door_handle_detector::DoorsDetector::Response res_handledetect;
  req_handledetect.door = door_in;
  if (!ros::service::call("door_handle_vision_detector", req_handledetect, res_handledetect)){
    ROS_ERROR("failed to detect a handle using the camera");
    return false;
  }

  door_out = res_handledetect.doors[0];
  return true;
}
