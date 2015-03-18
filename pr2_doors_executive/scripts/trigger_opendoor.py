#! /usr/bin/env python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#         * Redistributions of source code must retain the above copyright
#             notice, this list of conditions and the following disclaimer.
#         * Redistributions in binary form must reproduce the above copyright
#             notice, this list of conditions and the following disclaimer in the
#             documentation and/or other materials provided with the distribution.
#         * Neither the name of the Willow Garage, Inc. nor the names of its
#             contributors may be used to endorse or promote products derived from
#             this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Wim Meeussen



import roslib; roslib.load_manifest('pr2_doors_executive')
import rospy

from actionlib import SimpleActionClient
from door_msgs.msg import DoorAction, DoorGoal, Door


def main():
    rospy.init_node('trigger_open_door')

    # initial door
    prior_door = Door()
    prior_door.frame_p1.x = 1.0
    prior_door.frame_p1.y = -0.5
    prior_door.frame_p2.x = 1.0
    prior_door.frame_p2.y = 0.5
    prior_door.door_p1.x = 1.0
    prior_door.door_p1.y = -0.5
    prior_door.door_p2.x = 1.0
    prior_door.door_p2.y = 0.5
    prior_door.travel_dir.x = 1.0
    prior_door.travel_dir.y = 0.0
    prior_door.travel_dir.z = 0.0
    prior_door.rot_dir = Door.ROT_DIR_COUNTERCLOCKWISE
    prior_door.hinge = Door.HINGE_P2
    prior_door.header.frame_id = "base_footprint"

    door = DoorGoal()
    door.door = prior_door

    ac = SimpleActionClient('move_through_door', DoorAction)
    print 'Waiting for open door action server'
    ac.wait_for_server()
    print 'Sending goal to open door action server'
    ac.send_goal_and_wait(door, rospy.Duration(500.0), rospy.Duration(2.0))
    
if __name__ == '__main__':
    main()
