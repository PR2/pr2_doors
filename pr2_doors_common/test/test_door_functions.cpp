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
 * $Id: test_executive_functions.cpp 14649 2009-04-30 01:13:48Z meeussen $
 *
 *********************************************************************/

/* Author: Wim Meeussen */

#include <door_msgs/Door.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include "pr2_doors_common/door_functions.h"


using namespace ros;
using namespace std;
using namespace KDL;
using namespace pr2_doors_common;




int g_argc;
char** g_argv;

class TestEKF : public testing::Test
{
public:
  door_msgs::Door my_door_1, my_door_2, my_door_3, my_door_4, my_door_5, my_door_6;

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
    my_door_1.travel_dir.x = 1.0;
    my_door_1.travel_dir.y = 0.0;
    my_door_1.travel_dir.z = 0.0;
    my_door_1.rot_dir = door_msgs::Door::ROT_DIR_COUNTERCLOCKWISE;
    my_door_1.hinge = door_msgs::Door::HINGE_P2;
    my_door_1.header.frame_id = "base_footprint";
    
    my_door_2.frame_p2.x = 1.0;
    my_door_2.frame_p2.y = -0.5;
    my_door_2.frame_p1.x = 1.0;
    my_door_2.frame_p1.y = 0.5;
    my_door_2.door_p2.x = 1.0;
    my_door_2.door_p2.y = -0.5;
    my_door_2.door_p1.x = 1.0;
    my_door_2.door_p1.y = 0.5;
    my_door_2.travel_dir.x = 1.0;
    my_door_2.travel_dir.y = 0.0;
    my_door_2.travel_dir.z = 0.0;
    my_door_2.rot_dir = door_msgs::Door::ROT_DIR_COUNTERCLOCKWISE;
    my_door_2.hinge = door_msgs::Door::HINGE_P1;
    my_door_2.header.frame_id = "base_footprint";

    my_door_3.frame_p1.x = 1.0;
    my_door_3.frame_p1.y = -0.5;
    my_door_3.frame_p2.x = 1.0;
    my_door_3.frame_p2.y = 0.5;
    my_door_3.door_p1.x = 1.75;
    my_door_3.door_p1.y = -0.25;
    my_door_3.door_p2.x = 1.0;
    my_door_3.door_p2.y = 0.5;
    my_door_3.travel_dir.x = 1.0;
    my_door_3.travel_dir.y = 0.0;
    my_door_3.travel_dir.z = 0.0;
    my_door_3.rot_dir = door_msgs::Door::ROT_DIR_COUNTERCLOCKWISE;
    my_door_3.hinge = door_msgs::Door::HINGE_P2;
    my_door_3.header.frame_id = "base_footprint";

    my_door_4.frame_p2.x = 1.0;
    my_door_4.frame_p2.y = -0.5;
    my_door_4.frame_p1.x = 1.0;
    my_door_4.frame_p1.y = 0.5;
    my_door_4.door_p2.x = 1.75;
    my_door_4.door_p2.y = -0.25;
    my_door_4.door_p1.x = 1.0;
    my_door_4.door_p1.y = 0.5;
    my_door_4.travel_dir.x = 1.0;
    my_door_4.travel_dir.y = 0.0;
    my_door_4.travel_dir.z = 0.0;
    my_door_4.rot_dir = door_msgs::Door::ROT_DIR_COUNTERCLOCKWISE;
    my_door_4.hinge = door_msgs::Door::HINGE_P2;
    my_door_4.header.frame_id = "base_footprint";

    my_door_5.frame_p1.x = 1.0;
    my_door_5.frame_p1.y = -0.5;
    my_door_5.frame_p2.x = 1.0;
    my_door_5.frame_p2.y = 0.5;
    my_door_5.door_p1.x = 0.25;
    my_door_5.door_p1.y = -0.25;
    my_door_5.door_p2.x = 1.0;
    my_door_5.door_p2.y = 0.5;
    my_door_5.travel_dir.x = 1.0;
    my_door_5.travel_dir.y = 0.0;
    my_door_5.travel_dir.z = 0.0;
    my_door_5.rot_dir = door_msgs::Door::ROT_DIR_CLOCKWISE;
    my_door_5.hinge = door_msgs::Door::HINGE_P2;
    my_door_5.header.frame_id = "base_footprint";

    my_door_6.frame_p2.x = 1.0;
    my_door_6.frame_p2.y = -0.5;
    my_door_6.frame_p1.x = 1.0;
    my_door_6.frame_p1.y = 0.5;
    my_door_6.door_p2.x = 0.25;
    my_door_6.door_p2.y = -0.25;
    my_door_6.door_p1.x = 1.0;
    my_door_6.door_p1.y = 0.5;
    my_door_6.travel_dir.x = 1.0;
    my_door_6.travel_dir.y = 0.0;
    my_door_6.travel_dir.z = 0.0;
    my_door_6.rot_dir = door_msgs::Door::ROT_DIR_CLOCKWISE;
    my_door_6.hinge = door_msgs::Door::HINGE_P2;
    my_door_6.header.frame_id = "base_footprint";
  }
  

  /// Destructor
  ~TestEKF()
  {}
};



bool approx(double v1, double v2, double eps=0.01)
{
  return fabs(v1-v2) < eps;
}

bool approx(Vector v1, Vector v2, double eps=0.01)
{
  return approx(v1(0), v2(0), eps) && approx(v1(1), v2(1), eps) &&approx(v1(2), v2(2), eps);
}


TEST_F(TestEKF, test)
{
  cout << "door 1 " << my_door_1 << endl;
  ASSERT_TRUE(approx(getDoorAngle(my_door_1), 0));
  ASSERT_TRUE(approx(getFrameNormal(my_door_1), Vector(1,0,0)));
  ASSERT_TRUE(approx(getDoorNormal(my_door_1), Vector(1,0,0)));

  cout << "door 2 " << my_door_2 << endl;
  ASSERT_TRUE(approx(getDoorAngle(my_door_2), 0));
  ASSERT_TRUE(approx(getFrameNormal(my_door_2), Vector(1,0,0)));
  ASSERT_TRUE(approx(getDoorNormal(my_door_2), Vector(1,0,0)));

  cout << "door 3 " << my_door_3 << endl;
  Vector v3(1,1,0); v3.Normalize();
  ASSERT_TRUE(approx(getDoorAngle(my_door_3), M_PI/4.0));
  ASSERT_TRUE(approx(getFrameNormal(my_door_3), Vector(1,0,0)));
  ASSERT_TRUE(approx(getDoorNormal(my_door_3), v3));

  cout << "door 4 " << my_door_4 << endl;
  ASSERT_TRUE(approx(getDoorAngle(my_door_4), M_PI/4.0));
  ASSERT_TRUE(approx(getFrameNormal(my_door_4), Vector(1,0,0)));
  ASSERT_TRUE(approx(getDoorNormal(my_door_4), v3));

  cout << "door 5 " << my_door_5 << endl;
  Vector v5(1,-1,0); v5.Normalize();
  ASSERT_TRUE(approx(getDoorAngle(my_door_5), -M_PI/4.0));
  ASSERT_TRUE(approx(getFrameNormal(my_door_5), Vector(1,0,0)));
  ASSERT_TRUE(approx(getDoorNormal(my_door_5), v5));

  cout << "door 6 " << my_door_6 << endl;
  ASSERT_TRUE(approx(getDoorAngle(my_door_6), -M_PI/4.0));
  ASSERT_TRUE(approx(getFrameNormal(my_door_6), Vector(1,0,0)));
  ASSERT_TRUE(approx(getDoorNormal(my_door_6), v5));

  SUCCEED();
}





int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}
