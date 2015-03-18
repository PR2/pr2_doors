/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
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

/** \author Radu Bogdan Rusu */

#include <door_handle_detector/geometric_helper.h>

using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Get a set of point indices between some specified bounds
  * \param points a pointer to the point cloud message
  * \param indices the resultant set of indices
  * \param door_req a door request service containing the X-Y bounds in frame_p1 and frame_p2
  * \param tf a pointer to a TransformListener object
  * \param parameter_frame the TF frame ID in which min_z_bounds and max_z_bounds are given
  * \param min_z_bounds restrict the minimum search bounds on Z to this value
  * \param max_z_bounds restrict the maximum search bounds on Z to this value
  * \param frame_multiplier multiply the ||frame_p1-frame_p2|| distance by this number to wrap all possible situations in X-Y
  */
void
  obtainCloudIndicesSet (const sensor_msgs::PointCloud &points, vector<int> &indices, door_msgs::Door& door,
                         tf::TransformListener *tf, std::string parameter_frame,
                         double min_z_bounds, double max_z_bounds, double frame_multiplier)
{
  // frames used
  string cloud_frame = points.header.frame_id;
  string door_frame  = door.header.frame_id;

  // Resize the resultant indices to accomodate all data
  indices.resize (points.points.size ());

  // Transform the X-Y bounds from the door request service into the cloud TF frame
  tf::Stamped<geometry_msgs::Point32> frame_p1 (door.frame_p1, points.header.stamp, door_frame);
  tf::Stamped<geometry_msgs::Point32> frame_p2 (door.frame_p2, points.header.stamp, door_frame);
  transformPoint (tf, cloud_frame, frame_p1, frame_p1);
  transformPoint (tf, cloud_frame, frame_p2, frame_p2);

  ROS_INFO ("Start detecting door at points in frame %s [%g, %g, %g] -> [%g, %g, %g]",
            cloud_frame.c_str (), frame_p1.x, frame_p1.y, frame_p1.z, frame_p2.x, frame_p2.y, frame_p2.z);

  // Obtain the bounding box information in the reference frame of the laser scan
  geometry_msgs::Point32 min_bbox, max_bbox;

  if (frame_multiplier == -1)
  {
    ROS_INFO ("Door frame multiplier set to -1. Using the entire point cloud data.");
    // Use the complete bounds of the point cloud
    cloud_geometry::statistics::getMinMax (points, min_bbox, max_bbox);
    for (unsigned int i = 0; i < points.points.size (); i++)
      indices[i] = i;
  }
  else
  {
    // Transform the minimum/maximum Z bounds parameters from frame parameter_frame to the cloud TF frame
    min_z_bounds = transformDoubleValueTF (min_z_bounds, parameter_frame, cloud_frame, points.header.stamp, tf);
    max_z_bounds = transformDoubleValueTF (max_z_bounds, parameter_frame, cloud_frame, points.header.stamp, tf);
    ROS_INFO ("Capping Z-search using the door_min_z_bounds/door_max_z_bounds parameters in frame %s: [%g / %g]",
              cloud_frame.c_str (), min_z_bounds, max_z_bounds);

    // Obtain the actual 3D bounds
    get3DBounds (&frame_p1, &frame_p2, min_bbox, max_bbox, min_z_bounds, max_z_bounds, frame_multiplier);

    int nr_p = 0;
    for (unsigned int i = 0; i < points.points.size (); i++)
    {
      if ((points.points[i].x >= min_bbox.x && points.points[i].x <= max_bbox.x) &&
          (points.points[i].y >= min_bbox.y && points.points[i].y <= max_bbox.y) &&
          (points.points[i].z >= min_bbox.z && points.points[i].z <= max_bbox.z))
      {
        indices[nr_p] = i;
        nr_p++;
      }
    }
    indices.resize (nr_p);
  }


  ROS_INFO ("Number of points in bounds [%f,%f,%f] -> [%f,%f,%f]: %d.",
             min_bbox.x, min_bbox.y, min_bbox.z, max_bbox.x, max_bbox.y, max_bbox.z, (int)indices.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
  checkDoorEdges (const geometry_msgs::Polygon &poly, const geometry_msgs::Point32 &z_axis, double min_height, double eps_angle,
                  double &door_frame1, double &door_frame2)
{
  // Compute the centroid of the polygon
  geometry_msgs::Point32 centroid;
  cloud_geometry::nearest::computeCentroid (poly, centroid);

  // Divide into left and right
  std::vector<int> inliers_left, inliers_right;
  for (unsigned int i = 0; i < poly.points.size (); i++)
  {
    if (poly.points[i].x < centroid.x)
    {
      if (poly.points[i].y < centroid.y)
        inliers_left.push_back (i);
      else
        inliers_right.push_back (i);
    }
    else
    {
      if (poly.points[i].y < centroid.y)
        inliers_left.push_back (i);
      else
        inliers_right.push_back (i);
    }
  }

  door_frame1 = door_frame2 = 0.0;
  geometry_msgs::Point32 line_dir;
  // Parse over all the <left> lines defined by the polygon and check their length
  std::vector<geometry_msgs::Point32> new_points;
  for (unsigned int i = 0; i < inliers_left.size () - 1; i++)
  {
    // Check if the points are equal
    if (cloud_geometry::checkPointEqual (poly.points.at (inliers_left[i]), poly.points.at (inliers_left[i+1])))
      continue;
    // Check if there is a jump in the order of the points on the convex hull
    if (fabs (inliers_left[i] - inliers_left[i+1]) > 1)
      continue;

    // Get the line direction between points i and i+1
    line_dir.x = poly.points.at (inliers_left[i]).x - poly.points.at (inliers_left[i+1]).x;
    line_dir.y = poly.points.at (inliers_left[i]).y - poly.points.at (inliers_left[i+1]).y;
    line_dir.z = poly.points.at (inliers_left[i]).z - poly.points.at (inliers_left[i+1]).z;

    // Compute the angle between this direction and the Z axis
    double angle = cloud_geometry::angles::getAngle3D (line_dir, z_axis);
    if ( (angle < eps_angle) || ( (M_PI - angle) < eps_angle ) )
    {
      // Compute the length of the line
      double line_length = cloud_geometry::distances::pointToPointDistance (poly.points.at (inliers_left[i]), poly.points.at (inliers_left[i+1]));
      door_frame1 += line_length;

      new_points.push_back (poly.points.at (inliers_left[i]));
      new_points.push_back (poly.points.at (inliers_left[i+1]));
    }
  }

  // Parse over all the <right> lines defined by the polygon and check their length
  for (unsigned int i = 0; i < inliers_right.size () - 1; i++)
  {
    // Check if the points are equal
    if (cloud_geometry::checkPointEqual (poly.points.at (inliers_right[i]), poly.points.at (inliers_right[i+1])))
      continue;
    // Check if there is a jump in the order of the points on the convex hull
    if (fabs (inliers_right[i] - inliers_right[i+1]) > 1)
      continue;
    // Get the line direction between points i and i+1
    line_dir.x = poly.points.at (inliers_right[i]).x - poly.points.at (inliers_right[i+1]).x;
    line_dir.y = poly.points.at (inliers_right[i]).y - poly.points.at (inliers_right[i+1]).y;
    line_dir.z = poly.points.at (inliers_right[i]).z - poly.points.at (inliers_right[i+1]).z;

    // Compute the angle between this direction and the Z axis
    double angle = cloud_geometry::angles::getAngle3D (line_dir, z_axis);
    if ( (angle < eps_angle) || ( (M_PI - angle) < eps_angle ) )
    {
      // Compute the length of the line
      double line_length = cloud_geometry::distances::pointToPointDistance (poly.points.at (inliers_right[i]), poly.points.at (inliers_right[i+1]));
      door_frame2 += line_length;

      new_points.push_back (poly.points.at (inliers_right[i]));
      new_points.push_back (poly.points.at (inliers_right[i+1]));
    }
  }

  if (door_frame1 < min_height || door_frame2 < min_height || fabs (door_frame2 - door_frame1) > 2 * min (door_frame1, door_frame2))
    return (false);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Get the 3D bounds where the door will be searched for
  * \param p1 a point on the floor describing the door frame
  * \param p2 a point on the floor describing the door frame
  * \param min_b the minimum bounds (x-y-z)
  * \param max_b the maximum bounds (x-y-z)
  * \param min_z_bounds restrict the minimum search bounds on Z to this value
  * \param max_z_bounds restrict the maximum search bounds on Z to this value
  * \param multiplier multiply the ||p1-p2|| distance by this number to wrap all possible situations in X-Y
  */
void
  get3DBounds (geometry_msgs::Point32 *p1, geometry_msgs::Point32 *p2, geometry_msgs::Point32 &min_b, geometry_msgs::Point32 &max_b,
               double min_z_bounds, double max_z_bounds, int multiplier)
{
  // Get the door_frame distance in the X-Y plane
  float door_frame = sqrt ( (p1->x - p2->x) * (p1->x - p2->x) + (p1->y - p2->y) * (p1->y - p2->y) );

  float center[2];
  center[0] = (p1->x + p2->x) / 2.0;
  center[1] = (p1->y + p2->y) / 2.0;

  // Obtain the bounds (doesn't matter which is min and which is max at this point)
  min_b.x = center[0] + (multiplier * door_frame) / 2.0;
  min_b.y = center[1] + (multiplier * door_frame) / 2.0;
  min_b.z = min_z_bounds;

  max_b.x = center[0] - (multiplier * door_frame) / 2.0;
  max_b.y = center[1] - (multiplier * door_frame) / 2.0;
  max_b.z = max_z_bounds;

  // Order min/max
  if (min_b.x > max_b.x)
  {
    float tmp = min_b.x;
    min_b.x = max_b.x;
    max_b.x = tmp;
  }
  if (min_b.y > max_b.y)
  {
    float tmp = min_b.y;
    min_b.y = max_b.y;
    max_b.y = tmp;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Get the view point from where the scans were taken in the incoming PointCloud message frame
  * \param cloud_frame the point cloud message TF frame
  * \param viewpoint_cloud the resultant view point in the incoming cloud frame
  * \param tf a pointer to a TransformListener object
  */
void
  getCloudViewPoint (const string cloud_frame, geometry_msgs::PointStamped &viewpoint_cloud, const tf::TransformListener *tf)
{
  // Figure out the viewpoint value in the point cloud frame
  geometry_msgs::PointStamped viewpoint_laser;
  viewpoint_laser.header.frame_id = "laser_tilt_mount_link";
  // Set the viewpoint in the laser coordinate system to 0, 0, 0
  viewpoint_laser.point.x = viewpoint_laser.point.y = viewpoint_laser.point.z = 0.0;

  try
  {
    tf->transformPoint (cloud_frame, viewpoint_laser, viewpoint_cloud);
    ROS_INFO ("Cloud view point in frame %s is: %g, %g, %g.", cloud_frame.c_str (),
              viewpoint_cloud.point.x, viewpoint_cloud.point.y, viewpoint_cloud.point.z);
  }
  catch (tf::ConnectivityException)
  {
    ROS_WARN ("Could not transform a point from frame %s to frame %s!", viewpoint_laser.header.frame_id.c_str (), cloud_frame.c_str ());
    // Default to 0.05, 0, 0.942768
    viewpoint_cloud.point.x = 0.05; viewpoint_cloud.point.y = 0.0; viewpoint_cloud.point.z = 0.942768;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Get the best distribution statistics (mean and standard deviation) and select the inliers based on them
  * in a given channel space (specified by the d_idx index)
  * \param points a pointer to the point cloud message
  * \param indices a pointer to a set of point cloud indices to test
  * \param d_idx the dimension/channel index to test
  * \param inliers the resultant inliers
  */
void
selectBestDistributionStatistics (const sensor_msgs::PointCloud &points, const vector<int> &indices, int d_idx, vector<int> &inliers)
{
  double mean, stddev;
  // Compute the mean and standard deviation of the distribution
  cloud_geometry::statistics::getChannelMeanStd (points, indices, d_idx, mean, stddev);
  //ROS_INFO ("Mean and standard deviation of the %s channel (%d) distribution: %g / %g.", points->channels[d_idx].name.c_str (), d_idx, mean, stddev);

  // (Chebyshev's inequality: at least 98% of the values are within 7 standard deviations from the mean)
  vector<int> alpha_vals (71);
  int nr_a = 0;
  for (double alpha = 0; alpha < 7; alpha += .1)
  {
    cloud_geometry::statistics::selectPointsOutsideDistribution (points, indices, d_idx, mean, stddev, alpha,
                                                                 inliers);
    alpha_vals[nr_a] = inliers.size ();
    //ROS_INFO ("Number of points for %g: %d.", alpha, alpha_vals[nr_a]);
    nr_a++;
  }
  alpha_vals.resize (nr_a);

  // Compute the trimean of the distribution
  double trimean;
  cloud_geometry::statistics::getTrimean (alpha_vals, trimean);
  //ROS_INFO ("Trimean of the indices distribution: %g.", trimean);

  // Iterate over the list of alpha values to find the best one
  int best_i = 0;
  double best_alpha = DBL_MAX;
  for (unsigned int i = 0; i < alpha_vals.size (); i++)
  {
    double c_val = fabs ((double)alpha_vals[i] - trimean);
    if (c_val < best_alpha)       // Whenever we hit the same value, exit
    {
      best_alpha = c_val;
      best_i     = i;
    }
  }

  best_alpha = best_i / 10.0;
  //ROS_INFO ("Best alpha selected: %d / %d / %g", best_i, alpha_vals[best_i], best_alpha);

  // Select the inliers of the channel based on the best_alpha value
  cloud_geometry::statistics::selectPointsOutsideDistribution (points, indices, d_idx, mean, stddev, best_alpha, inliers);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Get the best dual distribution statistics (mean and standard deviation) and select the inliers based on
  * them in a given channel space (specified by the d_idx index)
  * \param points a pointer to the point cloud message
  * \param indices a pointer to a set of point cloud indices to test
  * \param d_idx_1 the first dimension/channel index to test
  * \param d_idx_2 the second dimension/channel index to test
  * \param inliers the resultant inliers
  */
void
  selectBestDualDistributionStatistics (const sensor_msgs::PointCloud &points, const vector<int> &indices, int d_idx_1, int d_idx_2,
                                        vector<int> &inliers)
{
  vector<int> inliers_1, inliers_2;
  double mean_1, stddev_1, mean_2, stddev_2;
  // Compute the mean and standard deviation of the distribution in the first dimension space
  cloud_geometry::statistics::getChannelMeanStd (points, indices, d_idx_1, mean_1, stddev_1);
  // Compute the mean and standard deviation of the distribution in the second dimension space
  cloud_geometry::statistics::getChannelMeanStd (points, indices, d_idx_2, mean_2, stddev_2);

  //for (int i = 0; i < indices->size (); i++)
  //  std::cout << points->channels[d_idx_1].values[indices->at (i)] << " " << points->channels[d_idx_2].values[indices->at (i)] << std::endl;

  // (Chebyshev's inequality: at least 98% of the values are within 7 standard deviations from the mean)
  vector<int> alpha_vals_1 (71), alpha_vals_2 (71);
  int nr_a = 0;
  for (double alpha = 0; alpha < 7; alpha += .1)
  {
    cloud_geometry::statistics::selectPointsOutsideDistribution (points, indices, d_idx_1, mean_1, stddev_1, alpha,
                                                                 inliers_1);
    cloud_geometry::statistics::selectPointsOutsideDistribution (points, indices, d_idx_2, mean_2, stddev_2, alpha,
                                                                 inliers_2);
    alpha_vals_1[nr_a] = inliers_1.size ();
    alpha_vals_2[nr_a] = inliers_2.size ();
    nr_a++;
  }
  alpha_vals_1.resize (nr_a);
  alpha_vals_2.resize (nr_a);

  //std::cout << "---------------------------" << endl;
  //for (int i = 0; i < alpha_vals_1.size (); i++)
  //  std::cout  << alpha_vals_1[i] << " " << alpha_vals_2[i] << std::endl;

  // Compute the trimean of the distribution
  double trimean_1, trimean_2;
  cloud_geometry::statistics::getTrimean (alpha_vals_1, trimean_1);
  cloud_geometry::statistics::getTrimean (alpha_vals_2, trimean_2);

  // Iterate over the list of alpha values to find the best one
  int best_i_1 = 0, best_i_2 = 0;
  double best_alpha_1 = DBL_MAX, best_alpha_2 = DBL_MAX;
  for (unsigned int i = 0; i < alpha_vals_1.size (); i++)
  {
    double c_val_1 = fabs ((double)alpha_vals_1[i] - trimean_1);
    if (c_val_1 < best_alpha_1)       // Whenever we hit the same value, exit
    {
      best_alpha_1 = c_val_1;
      best_i_1     = i;
    }
    double c_val_2 = fabs ((double)alpha_vals_2[i] - trimean_2);
    if (c_val_2 < best_alpha_2)       // Whenever we hit the same value, exit
    {
      best_alpha_2 = c_val_2;
      best_i_2     = 2;
    }
  }

  best_alpha_1 = best_i_1 / 10.0;
  best_alpha_2 = best_i_2 / 10.0;
  //ROS_INFO ("Best alpha selected: %d / %d / %g", best_i, alpha_vals[best_i], best_alpha);

  // Select the inliers of the channel based on the best_alpha value
  cloud_geometry::statistics::selectPointsOutsideDistribution (points, indices, d_idx_1, mean_1, stddev_1, best_alpha_1, inliers_1);
  cloud_geometry::statistics::selectPointsOutsideDistribution (points, indices, d_idx_2, mean_2, stddev_2, best_alpha_2, inliers_2);

  // Intersect the two inlier sets
  sort (inliers_1.begin (), inliers_1.end ());
  sort (inliers_2.begin (), inliers_2.end ());
  set_intersection (inliers_1.begin (), inliers_1.end (), inliers_2.begin (), inliers_2.end (), inserter (inliers, inliers.begin ()));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Test whether the given point indices are roughly sampled at the viewpoint <-> plane perpendicular
  * \param points a pointer to the point cloud message
  * \param indices a pointer to a set of point cloud indices to test
  * \param viewpoint the viewpoint of the laser when the cloud was acquired
  * \param coeff the plane coefficients
  * \param eps_angle a maximum allowed angular threshold
  */
bool
  checkIfClusterPerpendicular (const sensor_msgs::PointCloud &points, const vector<int> &indices, geometry_msgs::PointStamped *viewpoint,
                               vector<double> *coeff, double eps_angle)
{
  // Compute the centroid of the cluster
  geometry_msgs::Point32 centroid;
  cloud_geometry::nearest::computeCentroid (points, indices, centroid);

  // Create a line direction from the viewpoint to the centroid
  centroid.x -= viewpoint->point.x;
  centroid.y -= viewpoint->point.y;
  centroid.z -= viewpoint->point.z;

  // Compute the normal of this cluster
  Eigen::Vector4d plane_parameters;
  double curvature;
  cloud_geometry::nearest::computePointNormal (points, indices, plane_parameters, curvature);
  geometry_msgs::Point32 normal;
  normal.x = plane_parameters (0);
  normal.y = plane_parameters (1);
  normal.z = plane_parameters (2);

//  vector<double> z_axis (3, 0); z_axis[2] = 1.0;
  // Compute the angle between the Z-axis and the newly created line direction
//  double angle = acos (cloud_geometry::dot (&centroid, &z_axis));//coeff));
//  if (fabs (M_PI / 2.0 - angle) < eps_angle)
  double angle = cloud_geometry::angles::getAngle3D (centroid, normal);
  if ( (angle < eps_angle) || ( (M_PI - angle) < eps_angle ) )
    return (true);
  return (false);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Decompose a region of space into clusters based on the euclidean distance between points, and the normal
  * angular deviation
  * \NOTE: assumes normalized point normals !
  * \param points pointer to the point cloud message
  * \param indices pointer to a list of point indices
  * \param tolerance the spatial tolerance as a measure in the L2 Euclidean space
  * \param clusters the resultant clusters
  * \param nx_idx the index of the channel containing the X component of the normal
  * \param ny_idx the index of the channel containing the Y component of the normal
  * \param nz_idx the index of the channel containing the Z component of the normal
  * \param eps_angle the maximum allowed difference between normals in degrees for cluster/region growing
  * \param min_pts_per_cluster minimum number of points that a cluster may contain (default = 1)
  */
void
  findClusters (const sensor_msgs::PointCloud &points, const vector<int> &indices, double tolerance, vector<vector<int> > &clusters,
                int nx_idx, int ny_idx, int nz_idx,
                double eps_angle, unsigned int min_pts_per_cluster)
{
  // Create a tree for these points
  cloud_kdtree::KdTree* tree = new cloud_kdtree::KdTreeANN (points, indices);

  int nr_points = indices.size ();
  // Create a bool vector of processed point indices, and initialize it to false
  vector<bool> processed;
  processed.resize (nr_points, false);

  vector<int> nn_indices;
  vector<float> nn_distances;
  // Process all points in the indices vector
  for (int i = 0; i < nr_points; i++)
  {
    if (processed[i])
      continue;

    vector<int> seed_queue;
    int sq_idx = 0;
    seed_queue.push_back (i);

//    double norm_a = 0.0;
//    if (nx_idx != -1)         // If we use normal indices...
//      norm_a = sqrt (points->channels[nx_idx].values[indices->at (i)] * points->channels[nx_idx].values[indices->at (i)] +
//                     points->channels[ny_idx].values[indices->at (i)] * points->channels[ny_idx].values[indices->at (i)] +
//                     points->channels[nz_idx].values[indices->at (i)] * points->channels[nz_idx].values[indices->at (i)]);

    processed[i] = true;

    while (sq_idx < (int)seed_queue.size ())
    {
      // Search for sq_idx
      tree->radiusSearch (seed_queue.at (sq_idx), tolerance, nn_indices, nn_distances);

      for (unsigned int j = 1; j < nn_indices.size (); j++)       // nn_indices[0] should be sq_idx
      {
        if (processed.at (nn_indices[j]))                         // Has this point been processed before ?
          continue;

        processed[nn_indices[j]] = true;
        if (nx_idx != -1)                                         // Are point normals present ?
        {
//          double norm_b = sqrt (points.channels[nx_idx].values[indices.at (nn_indices[j])] * points.channels[nx_idx].values[indices.at (nn_indices[j])] +
//                                points.channels[ny_idx].values[indices.at (nn_indices[j])] * points.channels[ny_idx].values[indices.at (nn_indices[j])] +
//                                points.channels[nz_idx].values[indices.at (nn_indices[j])] * points.channels[nz_idx].values[indices.at (nn_indices[j])]);
          // [-1;1]
          double dot_p = points.channels[nx_idx].values[indices.at (i)] * points.channels[nx_idx].values[indices.at (nn_indices[j])] +
                         points.channels[ny_idx].values[indices.at (i)] * points.channels[ny_idx].values[indices.at (nn_indices[j])] +
                         points.channels[nz_idx].values[indices.at (i)] * points.channels[nz_idx].values[indices.at (nn_indices[j])];
//          if ( acos (dot_p / (norm_a * norm_b)) < eps_angle)
          if ( fabs (acos (dot_p)) < eps_angle )
          {
            processed[nn_indices[j]] = true;
            seed_queue.push_back (nn_indices[j]);
          }
        }
        // If normal information is not present, perform a simple Euclidean clustering
        else
        {
          processed[nn_indices[j]] = true;
          seed_queue.push_back (nn_indices[j]);
        }
      }

      sq_idx++;
    }

    // If this queue is satisfactory, add to the clusters
    if (seed_queue.size () >= min_pts_per_cluster)
    {
      vector<int> r (seed_queue.size ());
      for (unsigned int j = 0; j < seed_queue.size (); j++)
        r[j] = indices.at (seed_queue[j]);

      sort (r.begin (), r.end ());
      r.erase (unique (r.begin (), r.end ()), r.end ());

      clusters.push_back (r);
    }
  }

  // Destroy the tree
  delete tree;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Find a plane model in a point cloud given via a set of point indices with SAmple Consensus methods
  * \param points the point cloud message
  * \param indices a pointer to a set of point cloud indices to test
  * \param inliers the resultant planar inliers
  * \param coeff the resultant plane coefficients
  * \param viewpoint_cloud a point to the pose where the data was acquired from (for normal flipping)
  * \param dist_thresh the maximum allowed distance threshold of an inlier to the model
  * \param min_pts the minimum number of points allowed as inliers for a plane model
  */
bool
  fitSACPlane (sensor_msgs::PointCloud &points, vector<int> indices, vector<int> &inliers, vector<double> &coeff,
               const geometry_msgs::PointStamped &viewpoint_cloud, double dist_thresh, int min_pts)
{
  if ((int)indices.size () < min_pts)
  {
    inliers.resize (0);
    coeff.resize (0);
    return (false);
  }

  // Create and initialize the SAC model
  sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
  sample_consensus::SAC *sac             = new sample_consensus::RANSAC (model, dist_thresh);
  //sample_consensus::SAC *sac             = new sample_consensus::LMedS (model, dist_thresh);
  sac->setMaxIterations (500);
  model->setDataSet (&points, indices);

  // Search for the best plane
  if (sac->computeModel ())
  {
    // Obtain the inliers and the planar model coefficients
    if ((int)sac->getInliers ().size () < min_pts)
    {
      //ROS_ERROR ("fitSACPlane: Inliers.size (%d) < sac_min_points_per_model (%d)!", sac->getInliers ().size (), sac_min_points_per_model_);
      inliers.resize (0);
      coeff.resize (0);
      return (false);
    }
    sac->computeCoefficients (coeff);          // Compute the model coefficients
    sac->refineCoefficients (coeff);           // Refine them using least-squares
    model->selectWithinDistance (coeff, dist_thresh, inliers);
    //inliers = sac->getInliers ();

    cloud_geometry::angles::flipNormalTowardsViewpoint (coeff, points.points.at (inliers[0]), viewpoint_cloud);

    //ROS_DEBUG ("Found a model supported by %d inliers: [%g, %g, %g, %g]\n", sac->getInliers ().size (),
    //           coeff[0], coeff[1], coeff[2], coeff[3]);

    // Project the inliers onto the model
    model->projectPointsInPlace (inliers, coeff);
  }
  else
  {
    ROS_ERROR ("Could not compute a plane model.");
    return (false);
  }
  sort (inliers.begin (), inliers.end ());

  delete sac;
  delete model;
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Estimate point normals for a given point cloud message (in place)
  * \param points the original point cloud message
  * \param point_indices only use these point indices
  * \param points_down the downsampled point cloud message
  * \param k the number of nearest neighbors to search for
  * \param viewpoint_cloud a pointer to the viewpoint where the cloud was acquired from (used for normal flip)
  */
void
estimatePointNormals (const sensor_msgs::PointCloud &points, const vector<int> &point_indices, sensor_msgs::PointCloud &points_down, int k, const geometry_msgs::PointStamped &viewpoint_cloud)
{
  // Reserve space for 4 channels: nx, ny, nz, curvature
  points_down.channels.resize (4);
  points_down.channels[0].name = "nx";
  points_down.channels[1].name = "ny";
  points_down.channels[2].name = "nz";
  points_down.channels[3].name = "curvatures";
  for (unsigned int d = 0; d < points_down.channels.size (); d++)
    points_down.channels[d].values.resize (points_down.points.size ());

  // Create a Kd-Tree for the original cloud
  cloud_kdtree::KdTree *kdtree = new cloud_kdtree::KdTreeANN (points, point_indices);

  // Allocate enough space for point indices
  vector<vector<int> > points_k_indices (points_down.points.size ());
  vector<float> distances;

  // Get the nearest neighbors for all the point indices in the bounds
  for (int i = 0; i < (int)points_down.points.size (); i++)
  {
    //kdtree->nearestKSearch (i, k, points_k_indices[i], distances);
    kdtree->radiusSearch (points_down.points[i], 0.025, points_k_indices[i], distances);
  }

#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < (int)points_down.points.size (); i++)
  {
    // Compute the point normals (nx, ny, nz), surface curvature estimates (c)
    Eigen::Vector4d plane_parameters;
    double curvature;
    for (int j = 0; j < (int)points_k_indices[i].size (); j++)
      points_k_indices[i][j] = point_indices.at (points_k_indices[i][j]);
    cloud_geometry::nearest::computePointNormal (points, points_k_indices[i], plane_parameters, curvature);

    cloud_geometry::angles::flipNormalTowardsViewpoint (plane_parameters, points_down.points[i], viewpoint_cloud);

    points_down.channels[0].values[i] = plane_parameters (0);
    points_down.channels[1].values[i] = plane_parameters (1);
    points_down.channels[2].values[i] = plane_parameters (2);
    points_down.channels[3].values[i] = curvature;//fabs (plane_parameters (3));
  }
  // Delete the kd-tree
  delete kdtree;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Estimate point normals for a given point cloud message (in place)
  * \param points the original point cloud message
  * \param point_indices only use these point indices
  * \param k the number of nearest neighbors to search for
  * \param viewpoint_cloud a pointer to the viewpoint where the cloud was acquired from (used for normal flip)
  */
void
  estimatePointNormals (sensor_msgs::PointCloud &points, const vector<int> &point_indices, int k, const geometry_msgs::PointStamped &viewpoint_cloud)
{
  int old_channel_size = points.channels.size ();
  // Reserve space for 4 channels: nx, ny, nz, curvature
  points.channels.resize (old_channel_size + 4);
  points.channels[old_channel_size + 0].name = "nx";
  points.channels[old_channel_size + 1].name = "ny";
  points.channels[old_channel_size + 2].name = "nz";
  points.channels[old_channel_size + 3].name = "curvatures";
  for (unsigned int d = old_channel_size; d < points.channels.size (); d++)
    points.channels[d].values.resize (points.points.size ());

  // Create a Kd-Tree for the original cloud
  cloud_kdtree::KdTree *kdtree = new cloud_kdtree::KdTreeANN (points, point_indices);

  // Allocate enough space for point indices
  vector<vector<int> > points_k_indices (point_indices.size ());
  vector<float> distances;

  // Get the nearest neighbors for all the point indices in the bounds
  for (unsigned int i = 0; i < point_indices.size (); i++)
  {
    //kdtree->nearestKSearch (i, k, points_k_indices[i], distances);
    kdtree->radiusSearch (points.points[point_indices.at (i)], 0.025, points_k_indices[i], distances);
  }

#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < (int)point_indices.size (); i++)
  {
    // Compute the point normals (nx, ny, nz), surface curvature estimates (c)
    Eigen::Vector4d plane_parameters;
    double curvature;

    for (int j = 0; j < (int)points_k_indices[i].size (); j++)
      points_k_indices[i][j] = point_indices.at (points_k_indices[i][j]);

    cloud_geometry::nearest::computePointNormal (points, points_k_indices[i], plane_parameters, curvature);

    cloud_geometry::angles::flipNormalTowardsViewpoint (plane_parameters, points.points[point_indices.at (i)], viewpoint_cloud);

    points.channels[old_channel_size + 0].values[point_indices.at (i)] = plane_parameters (0);
    points.channels[old_channel_size + 1].values[point_indices.at (i)] = plane_parameters (1);
    points.channels[old_channel_size + 2].values[point_indices.at (i)] = plane_parameters (2);
    points.channels[old_channel_size + 3].values[point_indices.at (i)] = curvature; //fabs (plane_parameters (3));
  }
  // Delete the kd-tree
  delete kdtree;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Estimate point normals for a given point cloud message (in place)
  * \param points the original point cloud message
  * \param points_down the downsampled point cloud message
  * \param k the number of nearest neighbors to search for
  * \param viewpoint_cloud a pointer to the viewpoint where the cloud was acquired from (used for normal flip)
  */
void
  estimatePointNormals (const sensor_msgs::PointCloud &points, sensor_msgs::PointCloud &points_down, int k, const geometry_msgs::PointStamped &viewpoint_cloud)
{
  int nr_points = points_down.points.size ();
  // Reserve space for 4 channels: nx, ny, nz, curvature
  points_down.channels.resize (4);
  points_down.channels[0].name = "nx";
  points_down.channels[1].name = "ny";
  points_down.channels[2].name = "nz";
  points_down.channels[3].name = "curvatures";
  for (unsigned int d = 0; d < points_down.channels.size (); d++)
    points_down.channels[d].values.resize (nr_points);

  cloud_kdtree::KdTree *kdtree = new cloud_kdtree::KdTreeANN (points);

  // Allocate enough space for point indices
  vector<vector<int> > points_k_indices (nr_points);
  vector<float> distances;

  // Get the nearest neighbors for all the point indices in the bounds
  for (int i = 0; i < nr_points; i++)
  {
    //points_k_indices[i].resize (k);
    //kdtree->nearestKSearch (&points_down.points[i], k, points_k_indices[i], distances);
    //kdtree->radiusSearch (&points_down.points[i], 0.05, points_k_indices[i], distances);
    kdtree->radiusSearch (points_down.points[i], 0.025, points_k_indices[i], distances);
  }

//#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < nr_points; i++)
  {
    // Compute the point normals (nx, ny, nz), surface curvature estimates (c)
    Eigen::Vector4d plane_parameters;
    double curvature;
    cloud_geometry::nearest::computePointNormal (points, points_k_indices[i], plane_parameters, curvature);

    cloud_geometry::angles::flipNormalTowardsViewpoint (plane_parameters, points_down.points[i], viewpoint_cloud);

    points_down.channels[0].values[i] = plane_parameters (0);
    points_down.channels[1].values[i] = plane_parameters (1);
    points_down.channels[2].values[i] = plane_parameters (2);
    points_down.channels[3].values[i] = curvature;//fabs (plane_parameters (3));
  }
  // Delete the kd-tree
  delete kdtree;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Finds the best oriented line in points/indices with respect to a given axis and return the point inliers.
  * \param points a pointer to the point cloud message
  * \param indices the point cloud indices to use
  * \param dist_thresh maximum allowed distance threshold of an inlier point to the line model
  * \param axis the axis to check against
  * \param eps_angle maximum angular line deviation from the axis (in radians)
  * \param line_inliers the resultant point inliers
  */
int
  fitSACOrientedLine (sensor_msgs::PointCloud &points, const std::vector<int> &indices,
                      double dist_thresh, const geometry_msgs::Point32 &axis, double eps_angle, std::vector<int> &line_inliers)
{
  if (indices.size () < 2)
  {
    line_inliers.resize (0);
    return (-1);
  }

  // Create and initialize the SAC model
  sample_consensus::SACModelOrientedLine *model = new sample_consensus::SACModelOrientedLine ();
  sample_consensus::SAC *sac                    = new sample_consensus::RANSAC (model, dist_thresh);
  sac->setMaxIterations (100);
  model->setDataSet (&points, indices);
  model->setAxis (axis);
  model->setEpsAngle (eps_angle);

  vector<double> coeff;
  // Search for the best line
  if (sac->computeModel ())
  {
    sac->computeCoefficients (coeff);            // Compute the model coefficients
    //line_inliers = model->selectWithinDistance (sac->refineCoefficients (), dist_thresh);
    line_inliers = sac->getInliers ();
  }
  else
  {
    ROS_ERROR ("Could not compute an oriented line model.");
    return (-1);
  }

  sort (line_inliers.begin (), line_inliers.end ());
  delete sac;
  delete model;
  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Finds the best oriented line in points with respect to a given axis and return the point inliers.
  * \param points a pointer to the point cloud message
  * \param dist_thresh maximum allowed distance threshold of an inlier point to the line model
  * \param axis the axis to check against
  * \param eps_angle maximum angular line deviation from the axis (in radians)
  * \param line_inliers the resultant point inliers
  */
int
  fitSACOrientedLine (sensor_msgs::PointCloud &points,
                      double dist_thresh, const geometry_msgs::Point32 &axis, double eps_angle, std::vector<int> &line_inliers)
{
  if (points.points.size () < 2)
  {
    line_inliers.resize (0);
    return (-1);
  }

  // Create and initialize the SAC model
  sample_consensus::SACModelOrientedLine *model = new sample_consensus::SACModelOrientedLine ();
  sample_consensus::SAC *sac                    = new sample_consensus::RANSAC (model, dist_thresh);
  sac->setMaxIterations (100);
  model->setDataSet (&points);
  model->setAxis (axis);
  model->setEpsAngle (eps_angle);

  // Search for the best line
  vector<double> coeff;
  if (sac->computeModel ())
  {
    sac->computeCoefficients (coeff);             // Compute the model coefficients
    line_inliers = sac->getInliers ();
  }
  else
  {
    ROS_ERROR ("Could not compute an oriented line model.");
    return (-1);
  }
  sort (line_inliers.begin (), line_inliers.end ());
  delete sac;
  delete model;
  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Grows the current euclidean cluster with other points outside the plane
  * \param points a pointer to the point cloud message
  * \param indices the point cloud indices to use
  * \param cluster the point cloud cluster to grow
  * \param inliers the resultant point inliers
  * \param dist_thresh the distance threshold used
  */
void
  growCurrentCluster (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, const std::vector<int> &cluster,
                      std::vector<int> &inliers, double dist_thresh)
{
  // Copy the cluster
  inliers.resize (cluster.size ());
  for (unsigned int i = 0; i < cluster.size (); i++)
    inliers[i] = cluster.at (i);

  if (indices.size () < 2)
  {
    ROS_WARN ("[growCurrentCluster] Less than 2 points found in this cluster. Exiting...");
    return;
  }
  ROS_DEBUG ("[growCurrentCluster] Creating Kd-Tree with %d points for a %d-points cluster.", (int)indices.size (), (int)cluster.size ());

  // Create a Kd-Tree for the original cloud
  cloud_kdtree::KdTree *kdtree = new cloud_kdtree::KdTreeANN (points, indices);

  // Get the nearest neighbors for all the point indices in the bounds
  vector<float> distances;
  for (unsigned int i = 0; i < cluster.size (); i++)
  {
    vector<int> points_k_indices;

    kdtree->radiusSearch (points.points[cluster.at (i)], dist_thresh, points_k_indices, distances);
    // Copy the inliers
    if (points_k_indices.size () == 0)
      continue;
    int old_size = inliers.size ();
    inliers.resize (old_size + points_k_indices.size ());
    for (unsigned int j = 0; j < points_k_indices.size (); j++)
      inliers[old_size + j] = indices.at (points_k_indices[j]);
  }
  sort (inliers.begin (), inliers.end ());
  inliers.erase (unique (inliers.begin (), inliers.end ()), inliers.end ());

  // Delete the kd-tree
  delete kdtree;
}
