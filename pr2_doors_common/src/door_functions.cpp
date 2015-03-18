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

#include "pr2_doors_common/door_functions.h"
#include <kdl/frames.hpp>

using namespace ros;
using namespace std;
using namespace KDL;


namespace pr2_doors_common
{
  static const double eps_angle = 5.0*M_PI/180.0;
  static const double gripper_height = 0.8;

  tf::Stamped<tf::Pose> getRobotPose(const door_msgs::Door& door, double dist)
  {
    Vector x_axis(1,0,0);
    
    // get vector to center of frame
    Vector frame_1(door.frame_p1.x, door.frame_p1.y, door.frame_p1.z);
    Vector frame_2(door.frame_p2.x, door.frame_p2.y, door.frame_p2.z);
    Vector frame_center = (frame_2 + frame_1)/2.0;
    
    // get robot pos
    Vector frame_normal = getFrameNormal(door);
    Vector robot_pos = frame_center + (frame_normal * dist);
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.frame_id_ = door.header.frame_id;
    robot_pose.stamp_ = door.header.stamp;
    robot_pose.setOrigin( tf::Vector3(robot_pos(0), robot_pos(1), robot_pos(2)));
    robot_pose.setRotation( tf::createQuaternionFromRPY(getVectorAngle(x_axis, frame_normal), 0, 0) ); 
    
    return robot_pose;  
  }
  
  double getNearestDoorAngle(const tf::Pose& robot_pose, const door_msgs::Door& door, double robot_dist, double touch_dist)
  { 
    double angle = 0, step = 0;
    if (door.rot_dir == door.ROT_DIR_CLOCKWISE)
      step = -0.01;
    else if (door.rot_dir == door.ROT_DIR_COUNTERCLOCKWISE)
      step = 0.01;
    else{
      ROS_ERROR("Door rot dir is not specified");
      return 0.0;
    }

    while ((robot_pose.getOrigin() - getGripperPose(door, angle, touch_dist).getOrigin()).length() < robot_dist &&
           fabs(angle) < M_PI_2)
      angle += step;

    return angle;
  }

  tf::Stamped<tf::Pose> getGripperPose(const door_msgs::Door& door, double angle, double dist)
  {
    Vector x_axis(1,0,0);
    
    // get hinge point
    Vector hinge, frame_vec;
    if (door.hinge == door_msgs::Door::HINGE_P1){
      hinge = Vector(door.door_p1.x, door.door_p1.y, door.door_p1.z);
      frame_vec = Vector(door.frame_p2.x - door.frame_p1.x, door.frame_p2.y - door.frame_p1.y, door.frame_p2.z - door.frame_p1.z);
    }
    else if (door.hinge == door_msgs::Door::HINGE_P2){
      hinge = Vector(door.door_p2.x, door.door_p2.y, door.door_p2.z);
      frame_vec = Vector(door.frame_p1.x - door.frame_p2.x, door.frame_p1.y - door.frame_p2.y, door.frame_p1.z - door.frame_p2.z);
    }

    // get gripper pos
    frame_vec.Normalize();
    frame_vec = frame_vec * dist;
    Rotation rot_angle = Rotation::RotZ(angle);
    Vector gripper_pos = hinge + (rot_angle * frame_vec);
    
    tf::Stamped<tf::Pose> gripper_pose;
    Vector normal_frame = getFrameNormal(door);
    gripper_pose.frame_id_ = door.header.frame_id;
    gripper_pose.stamp_ = door.header.stamp;
    gripper_pose.setOrigin( tf::Vector3(gripper_pos(0), gripper_pos(1), gripper_height));
    gripper_pose.setRotation( tf::createQuaternionFromRPY(getVectorAngle(x_axis, normal_frame)+angle, 0, 0) ); 
    
    return gripper_pose;  
  }

  tf::Stamped<tf::Pose> getGripperPose(const door_msgs::Door& door, double angle, double dist, int side)
  {
    Vector x_axis(1,0,0);
    
    // get hinge point
    Vector hinge, frame_vec;
    if (door.hinge == door_msgs::Door::HINGE_P1){
      hinge = Vector(door.door_p1.x, door.door_p1.y, door.door_p1.z);
      frame_vec = Vector(door.frame_p2.x - door.frame_p1.x, door.frame_p2.y - door.frame_p1.y, door.frame_p2.z - door.frame_p1.z);
    }
    else if (door.hinge == door_msgs::Door::HINGE_P2){
      hinge = Vector(door.door_p2.x, door.door_p2.y, door.door_p2.z);
      frame_vec = Vector(door.frame_p1.x - door.frame_p2.x, door.frame_p1.y - door.frame_p2.y, door.frame_p1.z - door.frame_p2.z);
    }

    // get gripper pos
    frame_vec.Normalize();
    frame_vec = frame_vec * dist;
    Rotation rot_angle = Rotation::RotZ(angle);
    Vector gripper_pos = hinge + (rot_angle * frame_vec);
    
    tf::Stamped<tf::Pose> gripper_pose;
    Vector normal_frame = getFrameNormal(door);
    gripper_pose.frame_id_ = door.header.frame_id;
    gripper_pose.stamp_ = door.header.stamp;
    gripper_pose.setOrigin( tf::Vector3(gripper_pos(0), gripper_pos(1), gripper_height));
    if(side == door_msgs::DoorCmd::PULL)
    {
      gripper_pose.setRotation( tf::createQuaternionFromRPY(getVectorAngle(-x_axis, normal_frame)+angle, 0, 0) ); 
    }
    else
    {
      gripper_pose.setRotation( tf::createQuaternionFromRPY(getVectorAngle(x_axis, normal_frame)+angle, 0, 0) ); 
    }
    return gripper_pose;  
  }


  geometry_msgs::Point32 getHingeAxisPoint(const door_msgs::Door& door)
  {
    geometry_msgs::Point32 hinge;
    if (door.hinge == door_msgs::Door::HINGE_P1){
      hinge = door.door_p1;
    }
    else if (door.hinge == door_msgs::Door::HINGE_P2){
      hinge = door.door_p2;
    }
    return hinge;
  }

  geometry_msgs::Point32 getEdgePoint(const door_msgs::Door& door)
  {
    geometry_msgs::Point32 edge;
    if (door.hinge == door_msgs::Door::HINGE_P1){
      edge = door.door_p2;
    }
    else if (door.hinge == door_msgs::Door::HINGE_P2){
      edge = door.door_p1;
    }
    return edge;
  }

  tf::Stamped<tf::Pose> getHandlePose(const door_msgs::Door& door, int side)
  {
    Vector x_axis(1,0,0);
    
    double dist = sqrt(pow(door.frame_p1.x - door.handle.x,2)+pow(door.frame_p1.y - door.handle.y,2));
    if(door.hinge == door_msgs::Door::HINGE_P2)
    {
      dist = sqrt(pow(door.frame_p2.x - door.handle.x,2)+pow(door.frame_p2.y - door.handle.y,2));
    }
    double angle = getDoorAngle(door);

    // get hinge point
    Vector hinge, frame_vec;
    if (door.hinge == door_msgs::Door::HINGE_P1){
      hinge = Vector(door.door_p1.x, door.door_p1.y, door.door_p1.z);
      frame_vec = Vector(door.frame_p2.x - door.frame_p1.x, door.frame_p2.y - door.frame_p1.y, door.frame_p2.z - door.frame_p1.z);
    }
    else if (door.hinge == door_msgs::Door::HINGE_P2){
      hinge = Vector(door.door_p2.x, door.door_p2.y, door.door_p2.z);
      frame_vec = Vector(door.frame_p1.x - door.frame_p2.x, door.frame_p1.y - door.frame_p2.y, door.frame_p1.z - door.frame_p2.z);
    }

    // get gripper pos
    frame_vec.Normalize();
    frame_vec = frame_vec * dist;
    Rotation rot_angle = Rotation::RotZ(angle);
    Vector handle_pos = hinge + (rot_angle * frame_vec);
    
    tf::Stamped<tf::Pose> handle_pose;
    Vector normal_frame = getFrameNormal(door);
    if(side == -1)
    {
      normal_frame = -normal_frame;
    }
    handle_pose.frame_id_ = door.header.frame_id;
    handle_pose.stamp_ = door.header.stamp;
    handle_pose.setOrigin( tf::Vector3(handle_pos(0), handle_pos(1), gripper_height));
    handle_pose.setRotation( tf::createQuaternionFromRPY(getVectorAngle(x_axis, normal_frame)+angle, 0, 0) ); 
    
    tf::Pose gripper_rotate(tf::createQuaternionFromRPY(0.0,0.0,M_PI/2.0),tf::Vector3(0.0,0.0,0.0));
    handle_pose.mult(handle_pose,gripper_rotate);
    return handle_pose;  
  }

  double getDoorAngle(const door_msgs::Door& door)
  {
    Vector frame_vec(door.frame_p1.x-door.frame_p2.x, door.frame_p1.y-door.frame_p2.y, door.frame_p1.z-door.frame_p2.z);
    Vector door_vec(door.door_p1.x-door.door_p2.x, door.door_p1.y-door.door_p2.y, door.door_p1.z-door.door_p2.z);
    double angle = getVectorAngle(frame_vec, door_vec);

    // validity check
    if (door.rot_dir == door_msgs::Door::ROT_DIR_CLOCKWISE && angle > eps_angle)
      ROS_DEBUG("Door angle is positive, but door message specifies it turns clockwise");

    if (door.rot_dir == door_msgs::Door::ROT_DIR_COUNTERCLOCKWISE && angle < -eps_angle)
      ROS_DEBUG("Door angle is negative, but door message specifies it turns counter-clockwise");

    return angle;
  }

  double getVectorAngle(const Vector& v1, const Vector& v2)
  {
    Vector vec1 = v1; vec1.Normalize();
    Vector vec2 = v2; vec2.Normalize();
    double dot      = vec2(0) * vec1(0) + vec2(1) * vec1(1);
    double perp_dot = vec2(1) * vec1(0) - vec2(0) * vec1(1);
    return atan2(perp_dot, dot);
  }

  Vector getDoorNormal(const door_msgs::Door& door)
  {
    Vector frame_normal = getFrameNormal(door);
    Rotation rot_frame_door = Rotation::RotZ(getDoorAngle(door));
    return rot_frame_door * frame_normal;
  }

  Vector getFrameNormal(const door_msgs::Door& door)
  {
    // normal on frame
    Vector p12(door.frame_p1.x-door.frame_p2.x, door.frame_p1.y-door.frame_p2.y, door.frame_p1.z-door.frame_p2.z);
    p12.Normalize();
    Vector z_axis(0,0,1);
    Vector normal = p12 * z_axis;

    // make normal point in direction we travel through door
    Vector dir(door.travel_dir.x, door.travel_dir.y, door.travel_dir.z);
    if (dot(dir, normal) < 0)
      normal = normal * -1.0;

    return normal;
  }

  bool transformTo(const tf::Transformer& tf, const string& goal_frame, const door_msgs::Door& door_in, door_msgs::Door& door_out, const std::string& fixed_frame)
  {
    door_out = door_in;
    ros::Time time_now = ros::Time::now();
    if (!transformPointTo(tf, door_in.header.frame_id, goal_frame, door_in.header.stamp, door_in.frame_p1, door_out.frame_p1, fixed_frame, time_now)) return false;
    if (!transformPointTo(tf, door_in.header.frame_id, goal_frame, door_in.header.stamp, door_in.frame_p2, door_out.frame_p2, fixed_frame, time_now)) return false;
    if (!transformPointTo(tf, door_in.header.frame_id, goal_frame, door_in.header.stamp, door_in.door_p1, door_out.door_p1, fixed_frame, time_now)) return false;
    if (!transformPointTo(tf, door_in.header.frame_id, goal_frame, door_in.header.stamp, door_in.door_p2, door_out.door_p2, fixed_frame, time_now)) return false;
    if (!transformPointTo(tf, door_in.header.frame_id, goal_frame, door_in.header.stamp, door_in.handle, door_out.handle, fixed_frame, time_now)) return false;
    if (!transformVectorTo(tf, door_in.header.frame_id, goal_frame, door_in.header.stamp, door_in.travel_dir, door_out.travel_dir, fixed_frame, time_now)) return false;
    door_out.header.frame_id = goal_frame;
    door_out.header.stamp = time_now;
    return true;
  }

  bool transformPointTo(const tf::Transformer& tf, const string& source_frame, const string& goal_frame, const Time& time_source,
                        const geometry_msgs::Point32& point_in, geometry_msgs::Point32& point_out, const std::string& fixed_frame, const Time& time_goal)
  {
    ros::Duration timeout = Duration().fromSec(5.0);
    if (!tf.waitForTransform(source_frame, time_source, goal_frame, time_goal, fixed_frame, timeout)) return false;
    tf::Stamped<tf::Point> pnt(tf::Vector3(point_in.x, point_in.y, point_in.z), time_source, source_frame);
    tf.transformPoint(goal_frame, time_goal, pnt, fixed_frame, pnt);
    point_out.x = pnt[0];
    point_out.y = pnt[1];
    point_out.z = pnt[2];

    return true;
  }

  bool transformVectorTo(const tf::Transformer& tf, const string& source_frame, const string& goal_frame, const Time& time_source,
                         const geometry_msgs::Vector3& point_in, geometry_msgs::Vector3& point_out, const std::string& fixed_frame, const Time& time_goal)
  {
    ros::Duration timeout = Duration().fromSec(2.0);
    if (!tf.waitForTransform(source_frame, time_source, goal_frame, time_goal, fixed_frame, timeout)) return false;
    tf::Stamped<tf::Point> pnt(tf::Vector3(point_in.x, point_in.y, point_in.z), time_source, source_frame);
    tf.transformVector(goal_frame, time_goal, pnt, fixed_frame, pnt);
    point_out.x = pnt[0];
    point_out.y = pnt[1];
    point_out.z = pnt[2];

    return true;
  }

  std::vector<geometry_msgs::Point> getPolygon(const door_msgs::Door& door, const double &door_thickness)
  {
    std::vector<geometry_msgs::Point> door_polygon;
    geometry_msgs::Point tmp;
    Vector door_normal = getDoorNormal(door);
    tmp.x = door.frame_p1.x + door_normal(0) * door_thickness/2.0;
    tmp.y = door.frame_p1.y + door_normal(1) * door_thickness/2.0;
    door_polygon.push_back(tmp);

    tmp.x = door.frame_p1.x - door_normal(0) * door_thickness/2.0;
    tmp.y = door.frame_p1.y - door_normal(1) * door_thickness/2.0;
    door_polygon.push_back(tmp);

    tmp.x = door.frame_p2.x - door_normal(0) * door_thickness/2.0;
    tmp.y = door.frame_p2.y - door_normal(1) * door_thickness/2.0;
    door_polygon.push_back(tmp);

    tmp.x = door.frame_p2.x + door_normal(0) * door_thickness/2.0;
    tmp.y = door.frame_p2.y + door_normal(1) * door_thickness/2.0;
    door_polygon.push_back(tmp);
    return door_polygon;
  }

  door_msgs::Door rotateDoor(const door_msgs::Door& door, const double &angle)
  {
    door_msgs::Door result = door;
    geometry_msgs::Point32 hinge = door.frame_p1;
    geometry_msgs::Point32 edge = door.frame_p2;
    if(door.hinge == door.HINGE_P2)
    {
      hinge = door.frame_p2;
      edge = door.frame_p1;
    }
    double handle_distance = sqrt(pow(door.handle.y-hinge.y,2) + pow(door.handle.x-hinge.x,2)); //assumes handle is flush with door
    double door_frame_global_yaw = atan2(edge.y-hinge.y,edge.x-hinge.x);
    double door_length = sqrt(pow(edge.y-hinge.y,2) + pow(edge.x-hinge.x,2));

    double sth = sin(door_frame_global_yaw+angle);
    double cth = cos(door_frame_global_yaw+angle);

    geometry_msgs::Point32 new_edge;
    new_edge.x = hinge.x + cth *door_length;
    new_edge.y = hinge.y + sth *door_length;
    geometry_msgs::Point32 new_handle = door.handle;
    new_handle.x = hinge.x + cth*handle_distance;
    new_handle.y = hinge.y + sth*handle_distance;

    result.door_p1 = hinge;
    result.door_p2 = new_edge;
    if(door.hinge == door.HINGE_P2)
    {
      result.door_p2 = hinge;
      result.door_p1 = new_edge;
    }
    result.handle = new_handle;
    return result;
  }

  double getFrameAngle(const door_msgs::Door& door)
  {
    double result;
    if(door.hinge == door.HINGE_P1)
    {   
      result =  atan2(door.frame_p2.y - door.frame_p1.y, door.frame_p2.x - door.frame_p1.x);
    }
    else
    {
      result =  atan2(door.frame_p1.y - door.frame_p2.y, door.frame_p1.x - door.frame_p2.x);
    }
    return result;
  }

  double getHandleDir(const door_msgs::Door& d)
  {
    Vector normal = getFrameNormal(d);
    Vector frame_vec;
    if (d.hinge == door_msgs::Door::HINGE_P1)
      frame_vec = Vector(d.frame_p2.x - d.frame_p1.x, d.frame_p2.y - d.frame_p1.y, d.frame_p2.z - d.frame_p1.z);
    else if (d.hinge == door_msgs::Door::HINGE_P2)
      frame_vec = Vector(d.frame_p1.x - d.frame_p2.x, d.frame_p1.y - d.frame_p2.y, d.frame_p1.z - d.frame_p2.z);
    else
      ROS_ERROR("Hinge side is not defined");

    Vector rot_vec = frame_vec * normal;
    if (rot_vec(2) > 0)
      return -1.0;
    else
      return 1.0;
  }

  double getDoorDir(const door_msgs::Door& d)
  {
    // get frame vector
    Vector frame_vec;
    if (d.hinge == door_msgs::Door::HINGE_P1)
      frame_vec = Vector(d.frame_p2.x - d.frame_p1.x, d.frame_p2.y - d.frame_p1.y, d.frame_p2.z - d.frame_p1.z);
    else if (d.hinge == door_msgs::Door::HINGE_P2)
      frame_vec = Vector(d.frame_p1.x - d.frame_p2.x, d.frame_p1.y - d.frame_p2.y, d.frame_p1.z - d.frame_p2.z);
    else
      ROS_ERROR("Hinge side is not defined");

    // rotate frame vector
    if (d.rot_dir == door_msgs::Door::ROT_DIR_CLOCKWISE)
      frame_vec = Rotation::RotZ(-M_PI_2) * frame_vec;
    else if (d.rot_dir == door_msgs::Door::ROT_DIR_COUNTERCLOCKWISE)
      frame_vec = Rotation::RotZ(M_PI_2) * frame_vec;
    else
      ROS_ERROR("Rot dir is not defined");

    if (dot(frame_vec, getFrameNormal(d)) < 0)
      return -1.0; 
    else
      return 1.0;
  }

  std::ostream& operator<< (std::ostream& os, const door_msgs::Door& d)
  {
    os << "Door message in " << d.header.frame_id << " at time " << d.header.stamp.toSec() << endl;
    os << " - frame (" 
       << d.frame_p1.x << " " << d.frame_p1.y << " "<< d.frame_p1.z << ") -- (" 
       << d.frame_p2.x << " " << d.frame_p2.y << " "<< d.frame_p2.z << ")" << endl;

    os << " - door (" 
       << d.door_p1.x << " " << d.door_p1.y << " "<< d.door_p1.z << ") -- (" 
       << d.door_p2.x << " " << d.door_p2.y << " "<< d.door_p2.z << ")" << endl;

    os << " - handle (" 
       << d.handle.x << " " << d.handle.y << " "<< d.handle.z << ")" << endl;

    os << " - travel_dir (" 
       << d.travel_dir.x << " " << d.travel_dir.y << " "<< d.travel_dir.z << ")" << endl;

    os << " - latch_state " << d.latch_state << endl;
    os << " - hinge side " << d.hinge << endl;
    os << " - rot_dir " << d.rot_dir << endl;
    os << " - angle [deg] " << getDoorAngle(d)*180.0/M_PI << endl;
    return os;
  }


}
