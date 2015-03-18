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

#include <door_msgs/Door.h>
#include "pr2_doors_actions/action_detect_door.h"
#include "pr2_doors_actions/action_detect_handle.h"
#include "pr2_doors_actions/action_check_path.h"
#include "pr2_doors_actions/action_grasp_handle.h"
#include "pr2_doors_actions/action_open_door.h"
//#include "pr2_doors_actions/action_push_door.h"
#include "pr2_doors_actions/action_release_handle.h"
//#include "pr2_doors_actions/action_touch_door.h"
#include "pr2_doors_actions/action_unlatch_handle.h"
#include "pr2_doors_actions/action_move_base_door.h"


using namespace door_handle_detector;


// -----------------------------------
//              MAIN
// -----------------------------------

int main(int argc, char** argv)
{
  ros::init(argc,argv,"door_domain_action_runer"); 

  tf::TransformListener tf;

  DetectDoorAction detect_door(tf);
  DetectHandleAction detect_handle(tf);
  //  TouchDoorAction touch(tf);
  //  PushDoorAction push(tf);
  GraspHandleAction grasp(tf);
  UnlatchHandleAction unlatch(tf);
  OpenDoorAction open(tf);
  ReleaseHandleAction release(tf);
  CheckPathAction check_path(tf);
  MoveBaseDoorAction move_base_door(tf);

  ros::spin();
  return 0;
}
