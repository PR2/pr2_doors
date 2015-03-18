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

#include <ros/node.h>
#include <door_msgs/Door.h>
#include <checkerboard_detector/ObjectDetection.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <door_handle_detector/DoorDetector.h>

using namespace checkerboard_detector;
using namespace tf;

class DoorCheckerboardDetectorNode : public ros::Node
{
  public:
    std::string listen_topic_, publish_topic_,frame_id_;
    checkerboard_detector::ObjectDetection checkerboard_msg_;
    double door_width_;
    double door_checkerboard_x_offset_, door_checkerboard_z_offset_, checkerboard_handle_z_offset_, checkerboard_handle_x_offset_;
    tf::TransformListener tf_; /**< Used to do transforms */
    door_msgs::Door door_msg_;

    DoorCheckerboardDetectorNode(std::string node_name):ros::Node(node_name),tf_(*this, false, ros::Duration(10))
    {
      this->param<std::string>("door_checkerboard_detector/listen_topic",listen_topic_,"/checkerdetector/ObjectDetection");
      this->param<std::string>("door_checkerboard_detector/publish_topic",publish_topic_,"door_location");
      
      this->param<std::string>("door_checkerboard_detector/frame_id",frame_id_,"base_link");

      this->param<double>("door_checkerboard_detector/door_width",door_width_,0.9);
      this->param<double>("door_checkerboard_detector/door_checkerboard_x_offset",door_checkerboard_x_offset_,0.073);
      this->param<double>("door_checkerboard_detector/door_checkerboard_z_offset",door_checkerboard_z_offset_,1.47);

      this->param<double>("door_checkerboard_detector/checkerboard_handle_z_offset",checkerboard_handle_z_offset_,0.0);
      this->param<double>("door_checkerboard_detector/checkerboard_handle_x_offset",checkerboard_handle_x_offset_,0.0);

      subscribe(listen_topic_, checkerboard_msg_,  &DoorCheckerboardDetectorNode::doorCallback,1);
      advertise<door_msgs::Door>(publish_topic_,1);
      advertiseService("door_handle_checkerboard_detector", &DoorCheckerboardDetectorNode::detectDoor, this);

    }

    ~DoorCheckerboardDetectorNode()
    {
      unadvertise(publish_topic_);
      unadvertiseService("door_handle_checkerboard_detector");
      unsubscribe(listen_topic_);
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

    bool detectDoor(door_handle_detector::DoorDetector::Request &req, door_handle_detector::DoorDetector::Response &resp)
    {
      resp.door = door_msg_;
      return true;
    }

    void doorCallback()
    {

      tf::Pose door_pose;
      Stamped<tf::Point> frame_p1, frame_p2, door_p1, door_p2, handle;
      Stamped<tf::Point> target;

      std::string dummy = "";

      frame_p1.frame_id_ = checkerboard_msg_.header.frame_id;
      frame_p2.frame_id_ = checkerboard_msg_.header.frame_id;
      door_p1.frame_id_ = checkerboard_msg_.header.frame_id;
      door_p2.frame_id_ = checkerboard_msg_.header.frame_id;
      handle.frame_id_ = checkerboard_msg_.header.frame_id;

      if(checkerboard_msg_.get_objects_size() > 0)
      {
        door_msg_.header.stamp    = checkerboard_msg_.header.stamp;
        door_msg_.header.frame_id = frame_id_;
        tf::poseMsgToTF(checkerboard_msg_.objects[0].pose,door_pose);

        door_p1 = Stamped<tf::Point>(door_pose*Point(-door_checkerboard_x_offset_,door_checkerboard_z_offset_,0.0),checkerboard_msg_.header.stamp, checkerboard_msg_.header.frame_id,dummy);
        door_p2 = Stamped<tf::Point>(door_pose*Point(door_width_ - door_checkerboard_x_offset_,door_checkerboard_z_offset_,0.0), checkerboard_msg_.header.stamp,  checkerboard_msg_.header.frame_id, dummy);

        ROS_INFO("Door to checkerboard offsets: %f %f",door_checkerboard_x_offset_,door_checkerboard_z_offset_);
        ROS_INFO("Checkerboard to handle offsets: %f %f",checkerboard_handle_x_offset_,checkerboard_handle_z_offset_);

        ROS_INFO("Transforming from %s to %s", door_p1.frame_id_.c_str(),frame_id_.c_str());
        ROS_INFO("door_p1 : %f %f %f", door_p1.m_floats[0], door_p1.m_floats[1], door_p1.m_floats[2]);
        ROS_INFO("checkerboard frame : %f %f %f", checkerboard_msg_.objects[0].pose.position.x, checkerboard_msg_.objects[0].pose.position.y,  checkerboard_msg_.objects[0].pose.position.z);
        frame_p1 = door_p1;
        frame_p2 = door_p2;
        sleep(1);
        handle = Stamped<tf::Point>(door_pose*Point(checkerboard_handle_x_offset_,checkerboard_handle_z_offset_,0.0),checkerboard_msg_.header.stamp, checkerboard_msg_.header.frame_id,dummy);

        transformTFPoint(frame_id_,door_p1,target);
        ROS_INFO("door_p1 transformed : %f %f %f", target.m_floats[0], target.m_floats[1], target.m_floats[2]);
        pointTFToMsg32(target,door_msg_.door_p1);

        transformTFPoint(frame_id_,door_p2,target);
        pointTFToMsg32(target,door_msg_.door_p2);

        transformTFPoint(frame_id_,frame_p1,target);
        pointTFToMsg32(target,door_msg_.frame_p1);

        transformTFPoint(frame_id_,frame_p2,target);
        pointTFToMsg32(target,door_msg_.frame_p2);

        transformTFPoint(frame_id_,handle,target);
        pointTFToMsg32(handle,door_msg_.handle);

        publish(publish_topic_,door_msg_);
      }
    }
};

int main(int argc, char** argv)
{
  ros::init(argc,argv); 

  DoorCheckerboardDetectorNode node("test_door_detection");

  try {
    node.spin();
  }
  catch(char const* e){
    std::cout << e << std::endl;
  }
  
  return(0);
}
