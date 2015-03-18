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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include <ros/node.h>
#include <checkerboard_detector/ObjectDetection.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <door_msgs/Door.h>
#include <std_msgs/String.h>
#include <door_handle_detector/DoorDetector.h>

#define MIN_DIST_THRESHOLD 0.1
#define MAX_COMPARE_TIME 20

using namespace checkerboard_detector;
using namespace tf;

class TestDoorDetectionNode : public ros::Node
{
  public:
    std::string frame_id_, joy_topic_;
    std_msgs::String joy_msg_;
    double door_width_;
    tf::TransformListener tf_; /**< Used to do transforms */

    door_msgs::Door door_msg_from_checkerboard_;
    door_msgs::Door door_msg_from_detector_;
    door_msgs::Door door_msg_to_detector_;

  TestDoorDetectionNode(std::string node_name):ros::Node(node_name),tf_(*this, false, ros::Duration(10))
    {
      this->param<std::string>("test_door_detection_node/joy_topic",joy_topic_,"annotation_msg");
      this->param<std::string>("test_door_detection_node/frame_id",frame_id_,"base_footprint");

      double tmp; int tmp2;
      param("~/door_frame_p1_x", tmp, 1.5); door_msg_to_detector_.frame_p1.x = tmp;
      param("~/door_frame_p1_y", tmp, -0.5);door_msg_to_detector_.frame_p1.y = tmp;
      param("~/door_frame_p2_x", tmp, 1.5); door_msg_to_detector_.frame_p2.x = tmp;
      param("~/door_frame_p2_y", tmp, 0.5); door_msg_to_detector_.frame_p2.y = tmp;
      param("~/door_hinge" , tmp2, -1); door_msg_to_detector_.hinge = tmp2;
      param("~/door_rot_dir" , tmp2, -1); door_msg_to_detector_.rot_dir = tmp2;
      door_msg_to_detector_.header.frame_id = frame_id_;

      subscribe(joy_topic_, joy_msg_, &TestDoorDetectionNode::joyCallback,1);
    }

    ~TestDoorDetectionNode()
    {
      unsubscribe(joy_topic_);
    }

    void transformTFPoint(const std::string &frame, const Stamped<tf::Point> &in, Stamped<tf::Point> &out)
    {
      try{
        tf_.transformPoint(frame,in,out);
      }
      catch(tf::LookupException& ex) {
        ROS_INFO("No Transform available Error\n");
        return;
      }
      catch(tf::ConnectivityException& ex) {
        ROS_INFO("Connectivity Error\n");
        return;
      }
      catch(tf::ExtrapolationException& ex) {
        ROS_INFO("Extrapolation Error %s\n", ex.what());
        return;
      }
      catch(tf::TransformException e) {
        return;
      }
    }

    void pointTFToMsg32(const tf::Vector3 &in, geometry_msgs::Point32 &out)
    {
      out.x = in.x();
      out.y = in.y();
      out.z = in.z();
    }

    void joyCallback()
    {
      std::string status_string;
      ROS_INFO("Joystick message: %s",joy_msg_.data.c_str());
      if (joy_msg_.data != std::string("detect_door"))
      { 
        return;
      }
      if(detectDoor(door_msg_to_detector_, door_msg_from_detector_))
	{
	  ROS_INFO("Door detected");
        if(detectDoorCheckerboard(door_msg_from_checkerboard_,door_msg_from_checkerboard_))
        {
          if(compareDoorMsgs(door_msg_from_detector_,door_msg_from_checkerboard_,status_string))
          {
	    //            ROS_INFO("%s",status_string.c_str());
          }
          else
          {
            ROS_INFO("Failed to match door messages from laser and checkerboard");
          }
        }
        else
        {
          ROS_WARN("Could not see checkerboard");
        }
      }
      else
      {
        ROS_ERROR("No door detected");
      }
        return;          
    }

    double distancePoints(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2)
    {
      return(sqrt(pow((p1.x-p2.x),2) + pow((p1.y-p2.y),2) + pow((p1.z-p2.z),2)));
    }
    double distancePointsXY(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2)
    {
      return(sqrt(pow((p1.x-p2.x),2) + pow((p1.y-p2.y),2)));
    }

    bool compareDoorMsgs(door_msgs::Door msg_1, door_msgs::Door msg_2, std::string &print_string)
    {
      if (fabs(msg_1.header.stamp.toSec() - msg_2.header.stamp.toSec()) > MAX_COMPARE_TIME)      //First check the times to make sure they are fairly recent w.r.t each other
      {
        ROS_ERROR("Comparison invalid since time stamps for the two door messages are %f seconds apart which is greater than max allowable difference %f", fabs(msg_1.header.stamp.toSec() - msg_2.header.stamp.toSec()), (double) MAX_COMPARE_TIME);
        return false;
      }

      if(msg_1.header.frame_id != msg_2.header.frame_id)      // Now compare the frame_ids
      {
        ROS_ERROR("Comparison invalid since the two door messages do not have the same frame id");
        return false;
      }
      bool door_result(false), handle_result(false);
      if(distancePoints(msg_1.door_p1, msg_2.door_p1) < MIN_DIST_THRESHOLD)
      {
        print_string += "door_p1 from the detector matches door_p1 from ground truth\n";
        if(distancePoints(msg_1.door_p2, msg_2.door_p2) < MIN_DIST_THRESHOLD)
        {
          door_result = true;
          print_string += "door_p2 from the detector matches door_p2 from ground truth\n";
        }
        else
        {
          print_string += "door_p2 from the detector does not match door_p2 from ground truth\n";
        }
      }      
      else if(distancePointsXY(msg_1.door_p2, msg_2.door_p1) < MIN_DIST_THRESHOLD)
      {
        print_string += "door_p2 from the detector matches door_p1 from ground truth\n";
        if(distancePointsXY(msg_1.door_p1, msg_2.door_p2) < MIN_DIST_THRESHOLD)
        {
          door_result = true;
          print_string += "door_p1 from the detector matches door_p2 from ground truth\n";
        }
        else
        {
          print_string += "door_p1 from the detector does not match door_p2 from ground truth\n";
        }
      }
      else
      {
        print_string += "No match found between detected door and ground truth door position\n";
      }

      if(distancePoints(msg_1.handle, msg_2.handle) < MIN_DIST_THRESHOLD)
      {
        handle_result = true;
        print_string += "handle location from the detector matches handle location from ground truth\n";
      }
      else
        print_string += "handle location from the detector does not match handle location from ground truth\n";
      ROS_INFO("%s",print_string.c_str());
      return (door_result);
    }


    bool detectDoor(const door_msgs::Door& door_estimate,  door_msgs::Door& door_detection)
    {
      door_handle_detector::DoorDetector::Request  req;
      door_handle_detector::DoorDetector::Response res;
      req.door = door_estimate;
      if (ros::service::call("door_handle_detector", req, res)){
        door_detection = res.door;
        return true;
      }
      else
        return false;
    }

    bool detectDoorCheckerboard(const door_msgs::Door& door_estimate,  door_msgs::Door& door_detection)
    {
      door_handle_detector::DoorDetector::Request  req;
      door_handle_detector::DoorDetector::Response res;
      req.door = door_estimate;
      if (ros::service::call("door_handle_checkerboard_detector", req, res)){
        door_detection = res.door;
        return true;
      }
      else
        return false;
    }

};

int main(int argc, char** argv)
{
  ros::init(argc,argv); 

  TestDoorDetectionNode node("test_detection_node");

  try {
    node.spin();
  }
  catch(char const* e){
    std::cout << e << std::endl;
  }
  
  return(0);
}
