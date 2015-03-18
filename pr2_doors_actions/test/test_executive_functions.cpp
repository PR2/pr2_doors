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
 *
 * $Id$
 *
 *********************************************************************/

/* Author: Wim Meeussen */

#include <door_msgs/Door.h>
#include <ros/node.h>
#include <gtest/gtest.h>
#include <door_handle_detector/door_functions.h>
#include "pr2_doors_actions/executive_functions.h"

using namespace ros;
using namespace std;
using namespace door_handle_detector;




int g_argc;
char** g_argv;

class TestEKF : public testing::Test
{
public:
  door_msgs::Door my_door_1, my_door_2;

protected:
  /// constructor
  TestEKF()
  {
    my_door_1.frame_p1.x = 1.0;
    my_door_1.frame_p1.y = -0.5;
    my_door_1.frame_p2.x = 1.0;
    my_door_1.frame_p2.y = 0.5;
    my_door_1.door_p1.x = 1.0;
    my_door_1.door_p1.y = -0.5;
    my_door_1.door_p2.x = 1.0;
    my_door_1.door_p2.y = 0.5;
    my_door_1.normal.x = 1.0;
    my_door_1.normal.y = 0.0;
    my_door_1.normal.z = 0.0;
    my_door_1.rot_dir = door_msgs::Door::ROT_DIR_COUNTERCLOCKWISE;
    my_door_1.hinge = door_msgs::Door::HINGE_P2;
    my_door_1.header.frame_id = "base_footprint";
    
    my_door_2.frame_p1.x = -0.198;
    my_door_2.frame_p1.y = -1.08;
    my_door_2.frame_p2.x = 0.76;
    my_door_2.frame_p2.y = -0.82;
    my_door_2.door_p1.x = -0.08;
    my_door_2.door_p1.y = -1.12;
    my_door_2.door_p2.x = 0.70;
    my_door_2.door_p2.y = -0.87;
    my_door_2.normal.x = 0.29;
    my_door_2.normal.y = -0.95;
    my_door_2.normal.z = 0.0;
    my_door_2.rot_dir = door_msgs::Door::ROT_DIR_COUNTERCLOCKWISE;
    my_door_2.hinge = door_msgs::Door::HINGE_P2;
    my_door_2.header.frame_id = "base_footprint";
  }
  

  /// Destructor
  ~TestEKF()
  {}
};




TEST_F(TestEKF, test)
{
  tf::Stamped<tf::Pose> pose;

  pose = getRobotPose(my_door_2, 0.6);
  cout << "pose = " << pose.getOrigin()[0] << ", "  <<pose.getOrigin()[1] << ", " <<pose.getOrigin()[2] << endl;

  pose = getGripperPose(my_door_2, M_PI/4.0, 0.4);
  cout << "door = " << my_door_2 << endl;
  cout << "pose = " << pose.getOrigin()[0] << ", "  <<pose.getOrigin()[1] << ", " <<pose.getOrigin()[2] << endl;

  pose = getRobotPose(my_door_1, 0.7);
  ASSERT_TRUE(pose.getOrigin()[0] == 0.3);
  ASSERT_TRUE(pose.getOrigin()[1] == 0.0);
  ASSERT_TRUE(pose.getOrigin()[2] == 0.0);
  SUCCEED();
}




int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}
