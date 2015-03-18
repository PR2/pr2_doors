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

#include <door_handle_detector/handle_detector.h>
#include <pr2_doors_common/door_functions.h>


using namespace std;
using namespace ros;
using namespace mapping_msgs;
using namespace door_msgs;
using namespace door_handle_detector;
using namespace pr2_doors_common;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HandleDetector::HandleDetector ()
  : node_tilde_("~")
{
  // ---[ Parameters regarding geometric constraints for the door/handle
  {
    node_tilde_.param ("parameter_frame", parameter_frame_, string ("base_footprint"));
    node_tilde_.param ("fixed_frame", fixed_frame_, string ("odom_combined"));

    node_tilde_.param ("handle_distance_door_max_threshold", handle_distance_door_max_threshold_, 0.3); // maximum handle distance from the door plane

    node_tilde_.param ("handle_min_height", handle_min_height_, 0.5);            // minimum height for a door handle: 0.5m
    node_tilde_.param ("handle_max_height", handle_max_height_, 1.22);           // [ADA] maximum height for a door handle: 1.22m
    ROS_DEBUG ("Using the following thresholds for handle detection [min height / max height]: %f / %f.", handle_min_height_, handle_max_height_);
  }

  z_axis_.x = 0; z_axis_.y = 0; z_axis_.z = 1;
  k_search_ = 10;                // 10 k-neighbors by default

  distance_from_door_margin_ = 0.02 * 0.02;
  min_plane_pts_             = 1000;               // 1000 points
  sac_distance_threshold_    = 0.015;              // 1.5 cm by default

  // Parameters regarding the thresholds for Euclidean region growing/clustering
  euclidean_cluster_angle_tolerance_    = angles::from_degrees (15.0);
  euclidean_cluster_min_pts_            = 4;                  // 4 points
  euclidean_cluster_distance_tolerance_ = 0.04;               // 4 cm

  node_tilde_.param ("input_cloud_topic", input_cloud_topic_, string ("/full_cloud"));
  detect_srv_ = node_.advertiseService ("handle_detector", &HandleDetector::detectHandleSrv, this);
  detect_cloud_srv_ = node_.advertiseService ("handle_detector_cloud", 
                                              &HandleDetector::detectHandleCloudSrv, this);
  vis_marker_pub_ = node_tilde_.advertise<visualization_msgs::Marker> ("visualization_marker", 100);
  handle_pol_pub_ = node_tilde_.advertise<PolygonalMap> ("handle_polygon", 1);
  handle_reg_pub_ = node_tilde_.advertise<sensor_msgs::PointCloud> ("handle_regions", 1);
  door_outliers_pub_ = node_tilde_.advertise<sensor_msgs::PointCloud> ("door_outliers", 1);

  global_marker_id_ = 1;
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool HandleDetector::detectHandle (const door_msgs::Door& door, sensor_msgs::PointCloud pointcloud,
                                   std::vector<door_msgs::Door>& result) const
{
  ROS_INFO ("HandleDetector: Start detecting handle in a point cloud of size %i", (int)pointcloud.points.size ());

  Time ts = Time::now();
  Duration duration;
  Duration timeout = Duration().fromSec(2.0);

  // Transform the PCD (Point Cloud Data) into the parameter_frame, and work there
  if (!tf_.waitForTransform(parameter_frame_, pointcloud.header.frame_id, pointcloud.header.stamp, timeout)){
    ROS_ERROR ("HandleDetector: Could not transform point cloud from frame '%s' to frame '%s' at time %f.",
               pointcloud.header.frame_id.c_str (), parameter_frame_.c_str (), pointcloud.header.stamp.toSec());
    return false;
  }
  tf_.transformPointCloud (parameter_frame_, pointcloud, pointcloud);
  ROS_INFO("HandleDetector: Pointcloud transformed to '%s'", parameter_frame_.c_str());


  // transform the door message into the parameter_frame, and work there
  Door door_tr;
  if (!transformTo(tf_, parameter_frame_, door, door_tr, fixed_frame_)){
     ROS_ERROR ("HandleDetector: Could not transform door message from frame '%s' to frame '%s'.",
                door.header.frame_id.c_str (), parameter_frame_.c_str ());
     return false;
   }
  ROS_INFO("HandleDetector: Door message transformed to '%s'", parameter_frame_.c_str());


  vector<int> tmp_indices;    // Used as a temporary indices array
  geometry_msgs::Point32 tmp_p;              // Used as a temporary point

  // Get the cloud viewpoint in the parameter frame
  geometry_msgs::PointStamped viewpoint_cloud;
  getCloudViewPoint (parameter_frame_, viewpoint_cloud, &tf_);

  // Get the rough planar coefficients
  geometry_msgs::Point32 pt;
  pt.x = (door_tr.door_p2.x + door_tr.door_p1.x) / 2.0;
  pt.y = (door_tr.door_p2.y + door_tr.door_p1.y) / 2.0;
  pt.z = (door_tr.door_p2.z + door_tr.door_p1.z) / 2.0 + door_tr.height / 2.0;
  vector<double> coeff (4);
  KDL::Vector door_normal = getDoorNormal(door_tr);
  geometry_msgs::Point32 door_normal_pnt;
  door_normal_pnt.x = door_normal(0);
  door_normal_pnt.y = door_normal(1);
  door_normal_pnt.z = door_normal(2);
  coeff[0] = door_normal(0);
  coeff[1] = door_normal(1);
  coeff[2] = door_normal(2);
  coeff[3] = - cloud_geometry::dot (door_normal_pnt, pt);

  // ---[ Optimization: select a subset of the points for faster processing
  // Select points close to the door plane (assumes door did not move since detection !)
  int nr_p = 0;
  vector<int> indices_in_bounds (pointcloud.points.size ());
  double dist_p1p2 = cloud_geometry::distances::pointToPointXYDistanceSqr (door_tr.door_p1, door_tr.door_p2);
  for (int i = pointcloud.points.size () - 1; i >= 0; --i)
  {
    if (pointcloud.points[i].z > handle_min_height_ && pointcloud.points[i].z < handle_max_height_ &&
        cloud_geometry::distances::pointToPlaneDistance (pointcloud.points[i], coeff) < handle_distance_door_max_threshold_)
    {
      double dist_p1 = cloud_geometry::distances::pointToPointXYDistanceSqr (pointcloud.points[i], door_tr.door_p1);
      double dist_p2 = cloud_geometry::distances::pointToPointXYDistanceSqr (pointcloud.points[i], door_tr.door_p2);
      if (dist_p1 < dist_p1p2 && dist_p1 > distance_from_door_margin_ && dist_p2 < dist_p1p2 && dist_p2 > distance_from_door_margin_)
      {
        indices_in_bounds[nr_p] = i;
        nr_p++;
      }
    }
  }
  indices_in_bounds.resize (nr_p);
  sort (indices_in_bounds.begin (), indices_in_bounds.end ());

  vector<int> inliers, outliers;
  // Find the actual door plane. If the door moved since detection, return false/exit
  if (!fitSACPlane (pointcloud, indices_in_bounds, inliers, coeff, viewpoint_cloud, sac_distance_threshold_, min_plane_pts_) || inliers.size () == 0)
  {
    ROS_ERROR ("Could not find a door planar model in the input data (%d points)! Exiting...", nr_p);
    return (false);
  }
  // Compute the convex hull of the door
  geometry_msgs::Polygon polygon, polygon_tr;
  cloud_geometry::areas::convexHull2D (pointcloud, inliers, coeff, polygon);

  // Create a polygonal representation on the X-Y plane (makes all isPointIn2DPolygon computations easier)
  Eigen::Matrix4d transformation;
  cloud_geometry::transforms::getPlaneToPlaneTransformation (coeff, z_axis_, 0, 0, 0, transformation);
  cloud_geometry::transforms::transformPoints (polygon.points, polygon_tr.points, transformation);

  // Get the outliers (we'll need them later)
  getDoorOutliers (indices_in_bounds, inliers, coeff, polygon, polygon_tr, transformation, outliers, pointcloud);

  // Get the handle candidates including the door itself
  vector<int> handle_indices;
  getHandleCandidates (indices_in_bounds, coeff, polygon, polygon_tr, transformation,
                       handle_indices, pointcloud, viewpoint_cloud);

  // Create Kd-Tree and estimate the point normals in the original point cloud for the points left
  estimatePointNormals (pointcloud, handle_indices, k_search_, viewpoint_cloud);

  // Find the handle by performing a composite segmentation in distance and intensity space for all points left
  // Select points outside the (mean +/- \alpha_ * stddev) distribution
  int curvature_idx = cloud_geometry::getChannelIndex (pointcloud, "curvatures");
  int intensity_idx = cloud_geometry::getChannelIndex (pointcloud, "intensities");
  if (intensity_idx == -1 || curvature_idx == -1)
  {
    ROS_ERROR ("Intensity (and/or curvature) channels not present in the point cloud! Exiting...");
    return (false);
  }
  selectBestDualDistributionStatistics (pointcloud, handle_indices, curvature_idx, intensity_idx, tmp_indices);
  // Check if any points were returned
  if (tmp_indices.size () == 0)
  {
    ROS_WARN ("No handle indices found !");
    return (false);
  }
  handle_indices = tmp_indices;
  ROS_DEBUG ("Number of candidate points for clustering in the dual intensity-curvature space: %d.", (int)handle_indices.size ());

  // Refine the remaining handle indices using the door outliers
  ROS_DEBUG (" -- handle indices before refinement: %d.", (int)handle_indices.size ());
  geometry_msgs::Point32 door_axis = cloud_geometry::cross (coeff, z_axis_);
  refineHandleCandidatesWithDoorOutliers (handle_indices, outliers, polygon, coeff, door_axis, door_tr, pointcloud);
  ROS_DEBUG (" -- handle indices after refinement: %d.", (int)handle_indices.size ());

  duration = ros::Time::now () - ts;

  // Assemble the reply
  geometry_msgs::Point32 min_h, max_h, handle_center;
  cloud_geometry::statistics::getLargestDiagonalPoints (pointcloud, handle_indices, min_h, max_h);
  handle_center.x = (min_h.x + max_h.x) / 2.0;
  handle_center.y = (min_h.y + max_h.y) / 2.0;
  handle_center.z = (min_h.z + max_h.z) / 2.0;
  //polygon.points.resize (2);
  //polygon.points[0] = min_h; polygon.points[1] = max_h;
  cout << "min_h = " << min_h.x << " " << min_h.y << " "<< min_h.z << endl;
  cout << "max_h = " << max_h.x << " " << max_h.y << " "<< max_h.z << endl;
  cout << "handle_center = " << handle_center.x << " " << handle_center.y << " "<< handle_center.z << endl;

  // Calculate the unsigned distance from the point to the plane
  double distance_to_plane = coeff[0] * handle_center.x + coeff[1] * handle_center.y + coeff[2] * handle_center.z + coeff[3] * 1;
  cout << "distance to plane = " << distance_to_plane << endl;




  // Output the point regions
  sensor_msgs::PointCloud cloud_regions;
  cloud_regions.channels.resize (1); cloud_regions.channels[0].name = "intensities";

  cloud_regions.header = pointcloud.header;
  cloud_regions.points.resize (0);
  cloud_regions.channels[0].values.resize (0);

  bool show_cluster = true;
  if (show_cluster)
  {
    for (unsigned int i = 0; i < handle_indices.size (); i++)
    {
      cloud_regions.points.push_back ( pointcloud.points[handle_indices[i]] );
      cloud_regions.channels[0].values.push_back ( pointcloud.channels[curvature_idx].values[handle_indices[i]] );
    }
  }
  else
  {
    cloud_regions.points.push_back (handle_center);
    cloud_regions.channels[0].values.push_back (9999);
  }

  //cloud_regions.points.push_back (door_tr.door_p1);
  //cloud_regions.channels[0].values.push_back (9999);
  //cloud_regions.points.push_back (door_tr.door_p2);
  //cloud_regions.channels[0].values.push_back (9999);

  handle_reg_pub_.publish (cloud_regions);

  // Publish the outliers
  handle_indices = outliers;
  cloud_regions.points.resize (0);
  cloud_regions.channels[0].values.resize (0);
  for (unsigned int i = 0; i < handle_indices.size (); i++)
  {
    cloud_regions.points.push_back ( pointcloud.points[handle_indices[i]] );
    cloud_regions.channels[0].values.push_back ( pointcloud.channels[curvature_idx].values[handle_indices[i]] );
  }
  door_outliers_pub_.publish (cloud_regions);

  PolygonalMap pmap;
  pmap.header = pointcloud.header;
  pmap.polygons.resize (1);         // Allocate space for the handle polygonal representation
  pmap.polygons[0] = polygon;
  handle_pol_pub_.publish (pmap);

  // Reply door message
  result.resize(1);
  result[0] = door_tr;
  result[0].handle = handle_center;
  if (!transformTo(tf_, fixed_frame_, result[0], result[0], fixed_frame_)){
    ROS_ERROR ("HandleDetector: Could not transform door message from frame '%s' to frame '%s'.",
               result[0].header.frame_id.c_str (), fixed_frame_.c_str ());
     return false;
  }
  ROS_INFO("HandleDetector: Door message transformed to '%s'", fixed_frame_.c_str());

  ROS_INFO ("Handle detected. Result in frame %s \n  Handle = [%f, %f, %f]. \n  Total time: %f.",
            result[0].header.frame_id.c_str (),
            result[0].handle.x, result[0].handle.y, result[0].handle.z,
            duration.toSec ());
  return (true);
}






//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HandleDetector::refineHandleCandidatesWithDoorOutliers (vector<int> &handle_indices, vector<int> &outliers,
                                                             const geometry_msgs::Polygon &polygon,
                                                             const vector<double> &coeff, const geometry_msgs::Point32 &door_axis,
                                                             const Door& door_prior,
                                                             sensor_msgs::PointCloud& pointcloud) const
{
  // Split the remaining candidates into into clusters and remove solitary points (< euclidean_cluster_min_pts_)
  vector<vector<int> > clusters;
  findClusters (pointcloud, handle_indices, euclidean_cluster_distance_tolerance_, clusters, -1, -1, -1, 0, euclidean_cluster_min_pts_);
  ROS_DEBUG ("Number of clusters for the handle candidate points: %d.", (int)clusters.size ());

  // Copy the clusters back to the indices
  handle_indices.resize (0);
  for (unsigned int i = 0; i < clusters.size (); i++)
  {
    int old_size = handle_indices.size ();
    handle_indices.resize (old_size + clusters[i].size ());
    for (unsigned int j = 0; j < clusters[i].size (); j++)
      handle_indices.at (old_size + j) = clusters[i][j];
  }

  // Go over each cluster, fit vertical lines to get rid of the points near the door edges in an elegant manner,
  // then consider the rest as true handle candidate clusters
  vector<vector<int> > line_inliers (clusters.size ());
  vector<vector<int> > inliers (clusters.size ());
  for (int i = 0; i < (int)clusters.size (); i++)
  {
        // One method to prune point clusters would be to fit vertical lines (Z parallel) and remove their inliers
#if 0
    //fitSACOrientedLine (pointcloud, clusters[i], sac_distance_threshold_, &z_axis_, euclidean_cluster_angle_tolerance_, line_inliers[i]);
    //set_difference (clusters[i].begin (), clusters[i].end (), line_inliers[i].begin (), line_inliers[i].end (),
    //                inserter (remaining_clusters[i], remaining_clusters[i].begin ()));
#endif
    // Grow the current cluster using the door outliers
    growCurrentCluster (pointcloud, outliers, clusters[i], inliers[i], 2 * euclidean_cluster_distance_tolerance_);

    sensor_msgs::PointCloud tmp_cloud;
    cloud_geometry::projections::pointsToPlane  (pointcloud, inliers[i], tmp_cloud, coeff);
    // Fit the best horizontal line through each cluster
    fitSACOrientedLine (tmp_cloud, sac_distance_threshold_ * 2, door_axis, euclidean_cluster_angle_tolerance_, line_inliers[i]);
    for (unsigned int j = 0; j < line_inliers[i].size (); j++)
      line_inliers[i][j] = inliers[i][line_inliers[i][j]];
  }

  double best_score = -FLT_MAX;
  int best_i = -1;
  // Check the elongation of the clusters
  for (unsigned int i = 0; i < line_inliers.size (); i++)
  {
    if (line_inliers[i].size () == 0)
      continue;

    geometry_msgs::Point32 min_h, max_h, mid;
    cloud_geometry::statistics::getLargestXYPoints (pointcloud, line_inliers[i], min_h, max_h);
    mid.x = (min_h.x + max_h.x)/2.0;  mid.y = (min_h.y + max_h.y)/2.0;

    // score for line lengh
    double length = sqrt ( (min_h.x - max_h.x) * (min_h.x - max_h.x) + (min_h.y - max_h.y) * (min_h.y - max_h.y) );
    double fit = ((double)(line_inliers[i].size())) / ((double)(inliers[i].size()));
    double score = fit - 3.0 * fabs (length - 0.15);
    ROS_INFO ("  Handle line cluster %d with %d inliers, has fit %g and length %g  --> %g.", i, (int)line_inliers[i].size (), fit, length, score);

    // score for side of the door
    double dist_to_side = 0;
    if (door_prior.hinge == Door::HINGE_P1)
      dist_to_side = cloud_geometry::distances::pointToPointXYDistance(door_prior.door_p2, mid);
    else if (door_prior.hinge == Door::HINGE_P2)
      dist_to_side = cloud_geometry::distances::pointToPointXYDistance(door_prior.door_p1, mid);
    else
      ROS_ERROR("HandleDetector: Door hinge side not defined");
    if (dist_to_side > 0.3)
      score = 0;
    else
      score /= fmin(0.0001, dist_to_side);
    ROS_INFO ("  Handle is found at %f [m] from the door side", dist_to_side);

    if (score > best_score)
    {
      best_score = score;
      best_i = i;
    }
  }

  if (best_i == -1)
  {
    ROS_ERROR ("All clusters rejected!");
    // Copy the extra inliers to handle_indices
    for (unsigned int i = 0; i < inliers.size (); i++)
    {
      if (inliers[i].size () == 0)
        continue;
      int old_size = handle_indices.size ();
      handle_indices.resize (old_size + inliers[i].size ());
      for (unsigned int j = 0; j < inliers[i].size (); j++)
        handle_indices.at (old_size + j) = inliers[i][j];
    }
  }
  else
  {
    ROS_INFO ("Selecting cluster %d.", best_i);

    // Copy the intensity/curvature cluster
    handle_indices.resize (clusters[best_i].size ());
    for (unsigned int j = 0; j < clusters[best_i].size (); j++)
      handle_indices[j] = clusters[best_i][j];

    // Copy the inliers cluster
    int old_size = handle_indices.size ();
    handle_indices.resize (old_size + line_inliers[best_i].size ());
    for (unsigned int j = 0; j < line_inliers[best_i].size (); j++)
      handle_indices[old_size + j] = line_inliers[best_i][j];
  }
  sort (handle_indices.begin (), handle_indices.end ());
  handle_indices.erase (unique (handle_indices.begin (), handle_indices.end ()), handle_indices.end ());

}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void  HandleDetector::getHandleCandidates (const vector<int> &indices, const vector<double> &coeff,
                                           const geometry_msgs::Polygon &polygon, const geometry_msgs::Polygon &polygon_tr,
                                           Eigen::Matrix4d transformation, vector<int> &handle_indices,
                                           sensor_msgs::PointCloud& pointcloud, geometry_msgs::PointStamped& viewpoint_cloud) const
{
  // Check the points in bounds for extra geometric constraints
  handle_indices.resize (indices.size ());

  // Install the basis for a viewpoint -> point line
  vector<double> viewpoint_pt_line (6);
  viewpoint_pt_line[0] = viewpoint_cloud.point.x;
  viewpoint_pt_line[1] = viewpoint_cloud.point.y;
  viewpoint_pt_line[2] = viewpoint_cloud.point.z;

  // Remove outliers around the door margin
  geometry_msgs::Point32 tmp_p;              // Used as a temporary point
  int nr_p = 0;
  geometry_msgs::Point32 pt;
  for (unsigned int i = 0; i < indices.size (); i++)
  {
    // Transform the point onto X-Y for faster checking inside the polygonal bounds
    double distance_to_plane;
    cloud_geometry::projections::pointToPlane (pointcloud.points.at (indices.at (i)), pt, coeff, distance_to_plane);
    if (distance_to_plane < 0)
      continue;
    cloud_geometry::transforms::transformPoint (pt, tmp_p, transformation);
    if (!cloud_geometry::areas::isPointIn2DPolygon (tmp_p, polygon_tr))        // Is the point's projection inside the door ?
      continue;

    // Close to the edges (3D)
    if (cloud_geometry::distances::pointToPolygonDistanceSqr (tmp_p, polygon_tr) < distance_from_door_margin_)
      continue;

    // Check whether the line viewpoint->point intersects the polygon
    viewpoint_pt_line[3] = pointcloud.points.at (indices.at (i)).x - viewpoint_cloud.point.x;
    viewpoint_pt_line[4] = pointcloud.points.at (indices.at (i)).y - viewpoint_cloud.point.y;
    viewpoint_pt_line[5] = pointcloud.points.at (indices.at (i)).z - viewpoint_cloud.point.z;
    // Normalize direction
    double n_norm = sqrt (viewpoint_pt_line[3] * viewpoint_pt_line[3] +
                          viewpoint_pt_line[4] * viewpoint_pt_line[4] +
                          viewpoint_pt_line[5] * viewpoint_pt_line[5]);
    viewpoint_pt_line[3] /= n_norm;
    viewpoint_pt_line[4] /= n_norm;
    viewpoint_pt_line[5] /= n_norm;

    // Check for the actual intersection
    geometry_msgs::Point32 viewpoint_door_intersection;
    if (!cloud_geometry::intersections::lineWithPlaneIntersection (coeff, viewpoint_pt_line, viewpoint_door_intersection))
    {
      ROS_WARN ("Line and plane are parallel (no intersections found between the line and the plane).");
          continue;
    }
    // Transform the point onto X-Y for faster checking inside the polygonal bounds
    cloud_geometry::projections::pointToPlane (viewpoint_door_intersection, pt, coeff);
    cloud_geometry::transforms::transformPoint (pt, tmp_p, transformation);
    if (!cloud_geometry::areas::isPointIn2DPolygon (tmp_p, polygon_tr))    // Is the viewpoint<->point intersection inside the door ?
      continue;

    // Save the point indices which satisfied all the geometric criteria so far
    handle_indices[nr_p++] = indices.at (i);
      } // end loop over points
  handle_indices.resize (nr_p);    // Resize to the actual value
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HandleDetector::getDoorOutliers (const vector<int> &indices, const vector<int> &inliers,
                                      const vector<double> &coeff, const geometry_msgs::Polygon &polygon,
                                      const geometry_msgs::Polygon &polygon_tr, Eigen::Matrix4d transformation,
                                      vector<int> &outliers, sensor_msgs::PointCloud& pointcloud) const
{
  vector<int> tmp_indices;    // Used as a temporary indices array
  geometry_msgs::Point32 tmp_p;              // Used as a temporary point
  set_difference (indices.begin (), indices.end (), inliers.begin (), inliers.end (),
                  inserter (outliers, outliers.begin ()));

#if 0
  // Install the basis for a viewpoint -> point line
  vector<double> viewpoint_pt_line (6);
  viewpoint_pt_line[0] = viewpoint_cloud.point.x;
  viewpoint_pt_line[1] = viewpoint_cloud.point.y;
  viewpoint_pt_line[2] = viewpoint_cloud.point.z;
#endif
  geometry_msgs::Point32 pt;
  tmp_indices.resize (outliers.size ());
  int nr_p = 0;
  for (unsigned int i = 0; i < outliers.size (); i++)
  {
    // Compute a projection on the plane
    double distance_to_plane;
    cloud_geometry::projections::pointToPlane (pointcloud.points.at (outliers[i]), pt, coeff, distance_to_plane);
    if (distance_to_plane < 0)
      continue;
    cloud_geometry::transforms::transformPoint (pt, tmp_p, transformation);
    if (!cloud_geometry::areas::isPointIn2DPolygon (tmp_p, polygon_tr))        // Is the point's projection inside the door ?
          continue;

    // Remove outliers around the door margin (close to the edges)
    if (cloud_geometry::distances::pointToPolygonDistanceSqr (tmp_p, polygon_tr) < distance_from_door_margin_)
      continue;

    tmp_indices[nr_p] = outliers[i];
    nr_p++;
  }
  tmp_indices.resize (nr_p);
  outliers = tmp_indices;

  // Split the remaining candidates into into clusters and remove the small clusters
  vector<vector<int> > clusters;
  if (outliers.size () > 0)
  {
    findClusters (pointcloud, outliers, euclidean_cluster_distance_tolerance_, clusters, -1, -1, -1, 0, euclidean_cluster_min_pts_);
    outliers.resize (0);
    for (unsigned int i = 0; i < clusters.size (); i++)
    {
      int old_size = outliers.size ();
      outliers.resize (old_size + clusters[i].size ());
      for (unsigned int j = 0; j < clusters[i].size (); j++)
        outliers[old_size + j] = clusters[i][j];
    }
  }
  else
    ROS_DEBUG ("[getDoorOutliers] No door plane outliers found.");
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool  HandleDetector::detectHandleSrv (door_handle_detector::DoorsDetector::Request &req,
                                       door_handle_detector::DoorsDetector::Response &resp)
{
  // receive a new laser scan
  num_clouds_received_ = 0;

  ros::Subscriber cloud_sub = node_.subscribe(input_cloud_topic_, 1, &HandleDetector::cloud_cb, this);
  ros::Duration tictoc = ros::Duration ().fromSec (0.1);
  while ((int)num_clouds_received_ < 1)
    tictoc.sleep ();
  cloud_sub.shutdown();

  return detectHandle(req.door, pointcloud_, resp.doors);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool HandleDetector::detectHandleCloudSrv (door_handle_detector::DoorsDetectorCloud::Request &req,
                                           door_handle_detector::DoorsDetectorCloud::Response &resp)
{
  return detectHandle(req.door, req.cloud, resp.doors);
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HandleDetector::cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud)
{
  pointcloud_ = *cloud;
  ROS_INFO ("Received %d data points in frame %s with %d channels (%s).", (int)pointcloud_.points.size (), pointcloud_.header.frame_id.c_str (),
            (int)pointcloud_.channels.size (), cloud_geometry::getAvailableChannels (pointcloud_).c_str ());
  num_clouds_received_++;
}



/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "handle_detector_node");

  HandleDetector p;

  ros::MultiThreadedSpinner s(2);
  s.spin();

  return (0);
}
/* ]--- */

