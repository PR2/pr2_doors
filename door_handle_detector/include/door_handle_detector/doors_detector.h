/*
 * Copyright (c) 2009 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: doors_detector.cpp 12908 2009-03-24 17:14:06Z meeussen $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b doors_detector detects and returns a list of door _candidates_ (no magic here!) from 3D point cloud data acquired using a tilting laser sensor.

This node should replace the older door_handle_detector node (which will be obsolete by the time this is finished).

\note Assumes the door frame points are given in the same coordinate system as the incoming point cloud message!

 **/

#ifndef DOORS_DETECTOR_H
#define DOORS_DETECTOR_H

// ROS core
#include <ros/ros.h>
#include <roslib/Header.h>
#include <mapping_msgs/PolygonalMap.h>

// Most of the geometric routines that contribute to the door finding job are located here
#include <door_handle_detector/geometric_helper.h>

// Include the service call type
#include <door_handle_detector/DoorsDetector.h>
#include <door_handle_detector/DoorsDetectorCloud.h>

#include <angles/angles.h>


namespace door_handle_detector{

class DoorDetector
{

public:
  DoorDetector ();
  ~DoorDetector (){};

  /** \brief Service call to detect doors*/
  bool detectDoorSrv (door_handle_detector::DoorsDetector::Request &req,
                      door_handle_detector::DoorsDetector::Response &resp);
  /** \brief Service call to detect doors*/
  bool detectDoorCloudSrv (door_handle_detector::DoorsDetectorCloud::Request &req,
                           door_handle_detector::DoorsDetectorCloud::Response &resp);

  ros::ServiceServer detect_srv_, detect_cloud_srv_;
  ros::Publisher viz_marker_pub_, door_frames_pub_, door_regions_pub_;

private:
  /** \brief This is the main door detection function */
  bool detectDoors(const door_msgs::Door& door, sensor_msgs::PointCloud pointcloud,
                   std::vector<door_msgs::Door>& result) const;

  /** \brief Main point cloud callback.*/
  void cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud);

  double distToHinge(const door_msgs::Door& door, geometry_msgs::Point32& pnt) const;

  ros::NodeHandle node_tilde_, node_;
  mutable int global_marker_id_;

  // parameters for callback function
  sensor_msgs::PointCloud pointcloud_;
  std::string input_cloud_topic_;
  unsigned int num_clouds_received_;
  geometry_msgs::Point32 z_axis_;
  tf::TransformListener tf_;

  std::string parameter_frame_, fixed_frame_;

  // Parameters regarding geometric constraints for the door/handle
  double door_min_height_, door_min_width_, door_max_height_, door_max_width_, door_min_z_, max_dist_from_prior_;

  // Parameters regarding the _fast_ normals/plane computation using a lower quality (downsampled) dataset
  double leaf_width_;
  double sac_distance_threshold_;
  double normal_angle_tolerance_;
  int k_search_;

  // Parameters for the euclidean clustering/cluster rejection
  double euclidean_cluster_angle_tolerance_, euclidean_cluster_distance_tolerance_;
  int euclidean_cluster_min_pts_;

  // Parameters for "rectangularity" constraints
  double rectangle_constrain_edge_height_;
  double rectangle_constrain_edge_angle_;

  // Prune door candidates
  double minimum_region_density_;
  double maximum_search_radius_, maximum_search_radius_limit_, maximum_scan_angle_limit_;
  double minimum_z_, maximum_z_;;




};


};

#endif
