#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: add_two_ints_client 3357 2009-01-13 07:13:05Z jfaustwg $

## Simple demo of a rospy service client that calls a service to add
## two integers. 

PKG = 'door_handle_detector' # this package name

import roslib; roslib.load_manifest(PKG) 

import sys
import os
import string

import rospy
from geometry_msgs.msg import Point, Point32
from door_msgs.msg import Door
from door_handle_detector.srv import DoorDetector



def detect_door(door_estimateion):
    # block until the door_handle_detector service is available
    print "Waiting for service...", rospy.resolve_name('door_handle_detector')
    rospy.wait_for_service('door_handle_detector')
    print "Service is available"
    try:
        # create a handle to the add_two_ints service
        find_door = rospy.ServiceProxy('door_handle_detector', Door)
        door_position = find_door(door_estimation)
        print "Request finished"
        return door_position
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e





def move_arm(x, y, z, rx, ry, rz, w):
    print "Create publisher to arm"
    pub = rospy.Publisher('/arm_trajectory/command', PoseStamped)
    
    m = PoseStamped()
    m.header.frame_id = 'odom_combined'
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z
    m.pose.orientation.x = rx
    m.pose.orientation.y = ry
    m.pose.orientation.z = rz
    m.pose.orientation.w = w

    print "Publish pose command to arm"
    pub.publish(m)
    rospy.init_node('pub', anonymous=True)





if __name__ == "__main__":
    
    door_estimate = Door()
    door_estimate.frame_p1.x = 1.5
    door_estimate.frame_p1.y = 0.5
    door_estimate.frame_p2.x = 1.5
    door_estimate.frame_p2.y = -0.5
    door_estimate.header.frame_id = "odom_combined"

     # find door
    #door_position = detect_door(door_estimate)
    door_position = door_estimate

    tmp = Point32()
    tmp.x = door_position.frame_p1.x - door_position.frame_p2.x
    tmp.y = door_position.frame_p1.y - door_position.frame_p2.y
    tmp.z = door_position.frame_p1.z - door_position.frame_p2.z
    print "finished"

    #print "Frame found at (%s, %s)  (%s, %s)"%(door_position.door.frame_p1.x, door_position.door.frame_p1.y, 
    #                                           door_position.door.frame_p2.x, door_position.door.frame_p2.y)
    #print "Door found at (%s, %s)  (%s, %s)"%(door_position.door.door_p1.x, door_position.door.door_p1.y, 
    #                                          door_position.door.door_p2.x, door_position.door.door_p2.y)
    #print "Handle found at (%s, %s)"%(door_position.door.handle.x, door_position.door.handle.y)


    
