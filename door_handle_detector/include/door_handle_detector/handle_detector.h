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
 * $Id$
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b handle_detector detects a handle on a given door.

\note Assumes the door frame points are given in the same coordinate system as the incoming point cloud message!

 **/

// ROS core
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <mapping_msgs/PolygonalMap.h>

// Most of the geometric routines that contribute to the door finding job are located here
#include <door_handle_detector/geometric_helper.h>

// Include the service call type
#include "door_handle_detector/DoorsDetector.h"
#include "door_handle_detector/DoorsDetectorCloud.h"

#include <angles/angles.h>

namespace door_handle_detector{

class HandleDetector
{
public:
  HandleDetector ();
  ~HandleDetector (){};

  /** \brief This is the main service callback: it gets called whenever a request to find a new handle is given   */
  bool detectHandleSrv (door_handle_detector::DoorsDetector::Request &req,
                        door_handle_detector::DoorsDetector::Response &resp);
  bool detectHandleCloudSrv (door_handle_detector::DoorsDetectorCloud::Request &req,
                             door_handle_detector::DoorsDetectorCloud::Response &resp);

  ros::ServiceServer detect_srv_, detect_cloud_srv_;
  ros::Publisher vis_marker_pub_, handle_pol_pub_, handle_reg_pub_, door_outliers_pub_;


private:
  /** \brief Refine the intensity/curvature handle indices with the door outliers
   *
   * \param handle_indices the handle indices
   * \param outliers the door outliers
   * \param polygon the polygonal bounds of the door
   * \param coeff the door planar coefficients
   * \param door_axis search for lines along this axis
   *
   * \note The following global parameters are used:
   *       cloud_tr_
   *       distance_from_door_margin_, euclidean_cluster_distance_tolerance_, euclidean_cluster_min_pts_
   */
  void refineHandleCandidatesWithDoorOutliers (std::vector<int> &handle_indices, std::vector<int> &outliers,
                                               const geometry_msgs::Polygon &polygon,
                                               const std::vector<double> &coeff, const geometry_msgs::Point32 &door_axis,
                                               const door_msgs::Door& door_prior,
                                               sensor_msgs::PointCloud& pointcloud) const;

  /** \brief Select all points that could represent a handle, including the door
   *
   * \param indices a pointer to all the point indices
   * \param coeff the door planar coefficients
   * \param polygon the polygonal bounds of the door
   * \param polygon_tr the polygonal bounds of the door in the X-Y plane
   * \param transformation the transformation between the door planar coefficients and the X-Y plane
   *
   * \param handle_indices the resultant handle indices
   *
   * \note In principle, this method could be fused with getDoorOutliers, but we want to keep them separate for now
   * for debugging purposes
   * \note The following global parameters are used:
   *       cloud_tr_, viewpoint_cloud_
   */
  void getHandleCandidates (const std::vector<int> &indices, const std::vector<double> &coeff,
                            const geometry_msgs::Polygon &polygon, const geometry_msgs::Polygon &polygon_tr,
                            Eigen::Matrix4d transformation, std::vector<int> &handle_indices,
                            sensor_msgs::PointCloud& pointcloud, geometry_msgs::PointStamped& viewpoint_cloud) const;

  /** \brief Select the door outliers that could represent a handle
   *
   * \param indices a pointer to all the point indices
   * \param inliers a pointer to the point indices which are inliers for the door plane
   * \param coeff the door planar coefficients
   * \param polygon the polygonal bounds of the door
   * \param polygon_tr the polygonal bounds of the door in the X-Y plane
   * \param transformation the transformation between the door planar coefficients and the X-Y plane
   *
   * \param outliers the resultant outliers
   *
   * \note The following global parameters are used:
   *       cloud_tr_, viewpoint_cloud_
   *       distance_from_door_margin_, euclidean_cluster_distance_tolerance_, euclidean_cluster_min_pts_
   */
  void getDoorOutliers (const std::vector<int> &indices, const std::vector<int> &inliers,
                        const std::vector<double> &coeff, const geometry_msgs::Polygon &polygon,
                        const geometry_msgs::Polygon &polygon_tr, Eigen::Matrix4d transformation,
                        std::vector<int> &outliers, sensor_msgs::PointCloud& pointcloud) const;

  /** \brief Main point cloud callback. */
  void cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud);



  bool detectHandle (const door_msgs::Door& door, sensor_msgs::PointCloud pointcloud,
                     std::vector<door_msgs::Door>& result) const;

  ros::NodeHandle node_tilde_, node_;

  // ROS messages
  sensor_msgs::PointCloud pointcloud_;
  geometry_msgs::Point32 z_axis_;


  tf::TransformListener tf_;

  std::string input_cloud_topic_, parameter_frame_, fixed_frame_;
  unsigned int num_clouds_received_;

  int k_search_;

  // Parameters regarding geometric constraints for the door/handle
  double handle_distance_door_max_threshold_;

  // ADA requirements with respect to the handle
  double handle_max_height_, handle_min_height_;

    // Parameters for the euclidean clustering/cluster rejection
  double euclidean_cluster_angle_tolerance_, euclidean_cluster_distance_tolerance_;
  int euclidean_cluster_min_pts_;

  double distance_from_door_margin_, min_plane_pts_, sac_distance_threshold_;

  int global_marker_id_;

}; // class
}
