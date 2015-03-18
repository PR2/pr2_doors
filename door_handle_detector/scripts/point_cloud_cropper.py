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

# Crop a point cloud to Z min/max, in a given frame.  This could be more
# widely useful, but it turns out out to be really slow in Python.

PKG = 'door_handle_detector' # this package name

import roslib; roslib.load_manifest(PKG) 

import sys
from optparse import OptionParser
import tf
import bullet

import rospy
from sensor_msgs.msg import PointCloud, ChannelFloat32, Point32
from geometry_msgs.msg import Point32

NAME='point_cloud_cropper'

class Cropper:
  def __init__(self, zmin, zmax, frame):
    self.zmin = zmin
    self.zmax = zmax
    self.frame = frame
    self.sub = rospy.Subscriber('full_cloud', PointCloud, self.cloud_cb)
    self.pub = rospy.Publisher('full_cloud_cropped', PointCloud)
    self.tf = tf.listener.TransformListener()
    rospy.init_node('cropper', anonymous=True)

  def cloud_cb(self, msg):
    print 'Received message with %d points'%(len(msg.points))
    newmsg = PointCloud(msg.header,[],[])
    for c in msg.chan:
      newmsg.channels.append(ChannelFloat32(c.name,[]))
    for i in range(0,len(msg.points)):
      if self.frame is None:
        p = msg.points[i]
      else:
        tfp = tf.PointStamped(bullet.Vector3(msg.points[i].x, msg.points[i].y, msg.points[i].z), 0, 0, msg.header.frame_id)
        tfpprime = self.tf.transform_point(self.frame, tfp)
        p = Point32(tfpprime.point.x(), tfpprime.point.y(), tfpprime.point.z())
      if p.z >= self.zmin and p.z <= self.zmax:
        newmsg.points.append(p)
        for j in range(0,len(msg.chan)):
          newmsg.channels[j].values.append(msg.channels[j].values[i])
    print 'Publishing message with %d points'%(len(newmsg.points))
    self.pub.publish(newmsg)

if __name__ == "__main__":

  parser = OptionParser(usage="usage: %prog [options]", prog=NAME)
  parser.add_option("", "--zmin",
                    dest="zmin", default=None,
                    help="Minimum z value to allow through")
  parser.add_option("", "--zmax",
                    dest="zmax", default=None,
                    help="Maximum z value to allow through")
  parser.add_option("-f", "--frame",
                    dest="frame", default=None,
                    help="Frame to transform to before cropping")

  (options, args) = parser.parse_args(sys.argv)
  if options.zmin is None or options.zmax is None:
    parser.error("must specify both zmin and zmax")
    parser.print_help()

  c = Cropper(float(options.zmin), float(options.zmax), options.frame)
  rospy.spin()
