/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Author: Brian Gerkey */

#include <cstdio>

#include "ros/ros.h"
#include "ros/console.h"

#include "tf/transform_listener.h"

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "geometry_msgs/Point32.h"

class Cropper
{
  public:
    Cropper(double zmin, double zmax, const std::string& frame_id)
    {
      zmin_ = zmin;
      zmax_ = zmax;
      frame_id_ = frame_id;

      sub_ = n.subscribe<sensor_msgs::PointCloud>("full_cloud", 50, &Cropper::cloudCallback, this);
      pub_ = n.advertise<sensor_msgs::PointCloud>("full_cloud_cropped", 1);
    }

    ~Cropper()
    {
    }
  private:
    ros::NodeHandle n;
    tf::TransformListener tf_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    double zmin_, zmax_;
    std::string frame_id_;

    void cloudCallback(const sensor_msgs::PointCloudConstPtr& cloud_in)
    {
      ROS_INFO("Received message with %d points", (int)cloud_in->points.size());
      sensor_msgs::PointCloud cloud_transformed;


      if (!tf_.waitForTransform(frame_id_, cloud_in->header.frame_id, cloud_in->header.stamp, ros::Duration(2.0))) return
      tf_.transformPointCloud(frame_id_, *cloud_in, cloud_transformed);

      sensor_msgs::PointCloud cloud_out;
      cloud_out.header = cloud_transformed.header;
      for(unsigned int j=0;j<cloud_transformed.channels.size();j++)
      {
        sensor_msgs::ChannelFloat32 c;
        c.name = cloud_transformed.channels[j].name;
        cloud_out.channels.push_back(c);
      }
      for(unsigned int i=0;i<cloud_transformed.points.size();i++)
      {
        if((cloud_transformed.points[i].z >= zmin_) &&
           (cloud_transformed.points[i].z <= zmax_))
        {
          cloud_out.points.push_back(cloud_transformed.points[i]);
          for(unsigned int j=0;j<cloud_transformed.channels.size();j++)
          {
            cloud_out.channels[j].values.push_back(cloud_transformed.channels[j].values[i]);
          }
        }
      }

      pub_.publish(cloud_out);
      ROS_INFO("Published message with %d points", (int)cloud_out.points.size());
    }
};

#define USAGE "USAGE point_cloud_cropper <zmin> <zmax> <frame>"

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "cropper", ros::init_options::AnonymousName);
  if (argc != 4)
  {
    puts(USAGE);
    return 1;
  }

  Cropper c(atof(argv[1]), atof(argv[2]), argv[3]);
  ros::spin();
  return 0;
}
