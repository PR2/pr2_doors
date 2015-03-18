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

#ifndef DOOR_FUNCTIONS_H
#define DOOR_FUNCTIONS_H

#include <door_msgs/Door.h>
#include <door_msgs/DoorCmd.h>
#include <tf/tf.h>
#include <string.h>
#include <kdl/frames.hpp>

namespace pr2_doors_common{

/// get robot and gripper pose
  tf::Stamped<tf::Pose> getRobotPose(const door_msgs::Door& door, double dist);
  tf::Stamped<tf::Pose> getGripperPose(const door_msgs::Door& door, double angle, double dist);
  tf::Stamped<tf::Pose> getGripperPose(const door_msgs::Door& door, double angle, double dist, int side);
  tf::Stamped<tf::Pose> getHandlePose(const door_msgs::Door& door, int side=1);
  double getNearestDoorAngle(const tf::Pose& robot_pose, const door_msgs::Door& door, double robot_dist, double touch_dist);

/// get the door angle
  double getDoorAngle(const door_msgs::Door& door);
  double getVectorAngle(const KDL::Vector& v1, const KDL::Vector& v2);
  KDL::Vector getDoorNormal(const door_msgs::Door& door);
  KDL::Vector getFrameNormal(const door_msgs::Door& door);
  double getFrameAngle(const door_msgs::Door& door);
  double getDoorDir(const door_msgs::Door& d);
  double getHandleDir(const door_msgs::Door& d);


/// convert door message from its original frame/time, to the goal frame at time::now.
    bool transformTo(const tf::Transformer& tf, const std::string& goal_frame, const door_msgs::Door& door_in, 
                     door_msgs::Door& door_out, const std::string& fixed_frame="odom_combined");
    bool transformPointTo(const tf::Transformer& tf, const std::string& source_frame, const std::string& goal_frame, const ros::Time& time_source,
                          const geometry_msgs::Point32& point_in, geometry_msgs::Point32& point_out, const std::string& fixed_frame, const ros::Time& time_goal);
    bool transformVectorTo(const tf::Transformer& tf, const std::string& source_frame, const std::string& goal_frame, const ros::Time& time_source,
                           const geometry_msgs::Vector3& point_in, geometry_msgs::Vector3& point_out, const std::string& fixed_frame, const ros::Time& time_goal);

    std::ostream& operator<< (std::ostream& os, const door_msgs::Door& d);
    std::vector<geometry_msgs::Point> getPolygon(const door_msgs::Door& door, const double &door_thickness);
    door_msgs::Door rotateDoor(const door_msgs::Door& door, const double &angle);
    geometry_msgs::Point32 getHingeAxisPoint(const door_msgs::Door& door);
    geometry_msgs::Point32 getEdgePoint(const door_msgs::Door& door);

}
#endif
