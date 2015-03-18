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
 *   * Neither the name of the Willow Garage nor the names of its
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

// Author: Marius Muja

#include <vector>
#include <fstream>
#include <sstream>
#include <time.h>
#include <iostream>
#include <iomanip>


#include "cv_bridge/CvBridge.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "stereo_msgs/DisparityImage.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PointStamped.h"
#include "door_msgs/Door.h"
#include "visualization_msgs/Marker.h"
#include "door_handle_detector/DoorsDetector.h"
#include "std_srvs/Empty.h"

#include <string>

// transform library
#include <tf/transform_listener.h>

#include <topic_synchronizer2/topic_synchronizer.h>

#include <boost/thread.hpp>

using namespace std;


template <typename T>
class IndexedIplImage
{
  public:
    IplImage* img_;
    T* p;

    IndexedIplImage(IplImage* img) : img_(img)
    {
      p = (T*)img_->imageData;
    }

    operator IplImage*()
    {
      return img_;
    }

    T at(int x, int y, int chan = 0)
    {
      return *(p+y*img_->width+x*img_->nChannels+chan);
    }

    T integral_sum(const CvRect &r)
    {
      return at(r.x+r.width,r.y+r.height)-at(r.x+r.width,r.y)-at(r.x,r.y+r.height)+at(r.x,r.y);
    }

};

class HandleDetector
{
  public:

    ros::NodeHandle nh_tilde_, nh_;

    sensor_msgs::ImageConstPtr limage_;
    sensor_msgs::ImageConstPtr rimage_;
    stereo_msgs::DisparityImageConstPtr dispimg_;
    sensor_msgs::CameraInfoConstPtr rcinfo_;

    sensor_msgs::CvBridge lbridge_;
    sensor_msgs::CvBridge rbridge_;
    sensor_msgs::CvBridge dbridge_;

  //	sensor_msgs::PointCloud cloud_fetch;
    sensor_msgs::PointCloudConstPtr cloud_;

    IplImage* left_;
    IplImage* right_;
    IplImage* disp_;
    IplImage* disp_clone_;



    ros::Subscriber left_image_sub_;
  //	ros::Subscriber left_caminfo_sub_;
  //	ros::Subscriber right_image_sub_;
  //	ros::Subscriber right_caminfo_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber dispimg_sub_;
  //	ros::Subscriber stereoinfo_sub_;

    ros::ServiceServer detect_service_;
    ros::ServiceServer preempt_service_;

    ros::Publisher marker_pub_;

    TopicSynchronizer sync_;

    tf::TransformListener tf_;


    // minimum height to look at (in base_footprint frame)
    double min_height_;
    // maximum height to look at (in base_footprint frame)
    double max_height_;
    // no. of frames to detect handle in
    int frames_no_;
    // display stereo images ?
    bool display_;

    bool preempted_;
    bool got_images_;


    double timeout_;
    ros::Time start_image_wait_;


    ros::CallbackQueue service_queue_;
    boost::thread service_thread_;
    boost::mutex data_lock_;
    boost::condition_variable data_cv_;

    CvHaarClassifierCascade* cascade;
    CvMemStorage* storage;

    HandleDetector()
      : nh_tilde_("~"), left_(NULL), right_(NULL), disp_(NULL), disp_clone_(NULL), sync_(&HandleDetector::syncCallback, this)
    {
        // define node parameters
    	nh_tilde_.param("min_height", min_height_, 0.8);
    	nh_tilde_.param("max_height", max_height_, 1.2);
    	nh_tilde_.param("frames_no", frames_no_, 10);
    	nh_tilde_.param("timeout", timeout_, 3.0);		// timeout (in seconds) until an image must be received, otherwise abort
    	nh_tilde_.param("display", display_, false);
      stringstream ss;
      ss << getenv("ROS_ROOT") << "/../ros-pkg/mapping/door_handle_detector/data/";
      string path = ss.str();
      string cascade_classifier;
      nh_tilde_.param<string>("cascade_classifier", cascade_classifier, path + "handles_data.xml");

      if(display_)
      {
        cvNamedWindow("left", CV_WINDOW_AUTOSIZE);
        //cvNamedWindow("right", CV_WINDOW_AUTOSIZE);
        cvNamedWindow("disparity", CV_WINDOW_AUTOSIZE);
        cvNamedWindow("disparity_original", CV_WINDOW_AUTOSIZE);
      }

      // load a cascade classifier
      loadClassifier(cascade_classifier);
      // invalid location until we get a detection

      marker_pub_ = nh_tilde_.advertise<visualization_msgs::Marker>("visualization_marker",1);

      ros::AdvertiseServiceOptions service_opts = ros::AdvertiseServiceOptions::create<door_handle_detector::DoorsDetector>("door_handle_vision_detector",
        boost::bind(&HandleDetector::detectHandleSrv, this, _1, _2),ros::VoidPtr(), &service_queue_);

//        detect_service_ = nh_.advertiseService("door_handle_vision_detector", &HandleDetector::detectHandleSrv, this);
      detect_service_ = nh_.advertiseService(service_opts);
      preempt_service_ = nh_.advertiseService("door_handle_vision_preempt", &HandleDetector::preempt, this);
      got_images_ = false;

      service_thread_ = boost::thread(boost::bind(&HandleDetector::serviceThread, this));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ~HandleDetector ()
    {
      // Clear the queue and terminate the thread
      service_queue_.clear ();
      service_queue_.disable ();
    	service_thread_.join ();
    }

  private:
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void 
      loadClassifier (string cascade_classifier)
    {
      //		subscribe("/door_detector/door_msg", door, &HandleDetector::door_detected,1);
      ROS_INFO ("Loading cascade classifier: %s", cascade_classifier.c_str ());
      cascade = (CvHaarClassifierCascade*)(cvLoad(cascade_classifier.c_str (), 0, 0, 0));
      if (!cascade)
        ROS_ERROR ("Cannot load cascade classifier\n");
      storage = cvCreateMemStorage (0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void 
      subscribeStereoData ()
    {
      left_image_sub_ = nh_.subscribe (nh_.resolveName("stereo")+"/left/image_rect", 1, sync_.synchronize(&HandleDetector::leftImageCallback, this));
//		right_caminfo_sub_ = nh_.subscribe(nh_.resolveName("stereo")+"/right/cam_info", 1, sync_.synchronize(&HandleDetector::rightCameraInfoCallback, this));
      cloud_sub_ = nh_.subscribe (nh_.resolveName("stereo")+"/points", 1, sync_.synchronize(&HandleDetector::cloudCallback, this));
      dispimg_sub_ = nh_.subscribe (nh_.resolveName("stereo")+"/disparity", 1, sync_.synchronize(&HandleDetector::dispimgCallback, this));
//		stereoinfo_sub_ = nh_.subscribe(nh_.resolveName("stereo")+"/stereo_info", 1, sync_.synchronize(&HandleDetector::stereoinfoCallback, this));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void 
      unsubscribeStereoData ()
    {
    	left_image_sub_.shutdown ();
//    	right_caminfo_sub_.shutdown();
    	cloud_sub_.shutdown ();
    	dispimg_sub_.shutdown ();
//    	stereoinfo_sub_.shutdown();
    	sync_.reset ();
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void 
      syncCallback ()
    {
      if (disp_!=NULL)
        cvReleaseImage (&disp_);
      if (dbridge_.fromImage (dispimg_->image))
      {
        disp_ = cvCreateImage(cvGetSize (dbridge_.toIpl ()), IPL_DEPTH_8U, 1);
        //ROS_ERROR(" debug123 %d channels %d %d ", dispimg_->dpp,dbridge_.toIpl()->nChannels,disp_->nChannels);
        if (dbridge_.toIpl()->nChannels == 3) // convert to mono first
        {
          IplImage* tmp_ = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);
          cvCvtColor(dbridge_.toIpl(),tmp_,CV_BGR2GRAY);
          cvCvtScale(tmp_, disp_, 4.0);
          cvReleaseImage(&tmp_);
        }
        else
          cvCvtScale(dbridge_.toIpl(), disp_, 4.0);
      }

      got_images_ = true;
      data_cv_.notify_all ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void 
      leftImageCallback (const sensor_msgs::Image::ConstPtr& image)
    {
      //ROS_INFO("got left image callback: %lld", image->header.stamp.toNSec());
      boost::unique_lock<boost::mutex> lock (data_lock_);

      limage_ = image;
      if (lbridge_.fromImage (*limage_, "bgr8"))
        left_ = lbridge_.toIpl ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void 
      dispimgCallback (const stereo_msgs::DisparityImage::ConstPtr& dinfo)
    {
      boost::unique_lock<boost::mutex> lock(data_lock_);
      //ROS_INFO("got dispimg callback: %lld", dinfo->header.stamp.toNSec());
      dispimg_ = dinfo;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void 
      cloudCallback (const sensor_msgs::PointCloud::ConstPtr& point_cloud)
    {
      boost::unique_lock<boost::mutex> lock (data_lock_);
      //ROS_INFO("got cloud callback: %lld", point_cloud->header.stamp.toNSec());
      cloud_ = point_cloud;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Analyze the disparity image that values should not be too far off from one another
    // Id  -- 8 bit, 1 channel disparity image
    // R   -- rectangular region of interest
    // vertical -- This is a return that tells whether something is on a wall (descending disparities) or not.
    // minDisparity -- disregard disparities less than this
    //
    double 
      disparitySTD(IplImage *Id, const CvRect & R, double & meanDisparity, double minDisparity = 0.5)
    {
      int ws = Id->widthStep;
      unsigned char *p = (unsigned char*)(Id->imageData);
      int rx = R.x;
      int ry = R.y;
      int rw = R.width;
      int rh = R.height;
      int nchan = Id->nChannels;
      p += ws * ry + rx * nchan; //Put at start of box
      double mean = 0.0, var = 0.0;
      double val;
      int cnt = 0;
      //For vertical objects, Disparities should decrease from top to bottom, measure that
      for(int Y = 0;Y < rh;++Y)
      {
        for(int X = 0;X < rw;X++, p += nchan)
        {
          val = (double)*p;
          if(val < minDisparity)
            continue;

          mean += val;
          var += val * val;
          cnt++;
        }
        p += ws - (rw * nchan);
      }

      if(cnt == 0)
        return (10000000.0);
      //DO THE VARIANCE MATH
      mean = mean / (double)cnt;
      var = (var / (double)cnt) - mean * mean;
      meanDisparity = mean;
      return (sqrt(var));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    struct Stats
    {
      Stats() : mean(0), stdev(0), min(0), max(0) {};

      float mean;
      float stdev;
      float min;
      float max;
    };


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void 
      pointCloudStatistics(const sensor_msgs::PointCloud& pc, Stats& x_stats, Stats& y_stats, Stats& z_stats)
    {
      uint32_t size = pc.get_points_size ();
      if (size == 0)
        return;
      x_stats.mean = 0;
      x_stats.stdev = 0;
      y_stats.mean = 0;
      y_stats.stdev = 0;
      z_stats.mean = 0;
      z_stats.stdev = 0;

      x_stats.min = pc.points[0].x;
      x_stats.max = pc.points[0].x;
      y_stats.min = pc.points[0].y;
      y_stats.max = pc.points[0].y;
      z_stats.min = pc.points[0].z;
      z_stats.max = pc.points[0].z;

      for (uint32_t i=0;i<size;++i) 
      {
        x_stats.mean += pc.points[i].x;
        y_stats.mean += pc.points[i].y;
        z_stats.mean += pc.points[i].z;
        x_stats.stdev += (pc.points[i].x*pc.points[i].x);
        y_stats.stdev += (pc.points[i].y*pc.points[i].y);
        z_stats.stdev += (pc.points[i].z*pc.points[i].z);

        if (x_stats.min>pc.points[i].x) x_stats.min = pc.points[i].x;
        if (x_stats.max<pc.points[i].x) x_stats.max = pc.points[i].x;
        if (y_stats.min>pc.points[i].y) y_stats.min = pc.points[i].y;
        if (y_stats.max<pc.points[i].y) y_stats.max = pc.points[i].y;
        if (z_stats.min>pc.points[i].z) z_stats.min = pc.points[i].z;
        if (z_stats.max<pc.points[i].z) z_stats.max = pc.points[i].z;
      }

      x_stats.mean /= size;
      y_stats.mean /= size;
      z_stats.mean /= size;

      x_stats.stdev = x_stats.stdev/size - x_stats.mean*x_stats.mean;
      y_stats.stdev = y_stats.stdev/size - y_stats.mean*y_stats.mean;
      z_stats.stdev = z_stats.stdev/size - z_stats.mean*z_stats.mean;
    }



    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * \brief Filters a cloud point, retains only points coming from a specific region in the disparity image
     *
     * @param rect Region in disparity image
     * @return Filtered point cloud
     */
    sensor_msgs::PointCloud 
      filterPointCloud (const sensor_msgs::PointCloud pc, const CvRect& rect)
    {
      sensor_msgs::PointCloud result;
      result.header.frame_id = pc.header.frame_id;
      result.header.stamp = pc.header.stamp;

      int xchan = -1;
      int ychan = -1;

      for (size_t i=0;i<pc.channels.size();++i) 
      {
        if (pc.channels[i].name == "u")
          xchan = i;
        if (pc.channels[i].name == "v") 
          ychan = i;
      }

      if (xchan == -1 || ychan == -1)
        return (result);

      int chan_size = pc.get_channels_size ();
      result.channels.resize (chan_size);
      for (int j = 0;j < chan_size; ++j)
        result.channels[j].name = pc.channels[j].name;

      for (size_t i = 0; i < pc.points.size (); ++i) 
      {
        int x = (int)pc.channels[xchan].values[i];
        int y = (int)pc.channels[ychan].values[i];
        if (x >= rect.x && x < rect.x + rect.width && y >= rect.y && y < rect.y + rect.height) 
        {
          result.points.push_back (pc.points[i]);
          for (int j = 0;j < chan_size; ++j) 
            result.channels[j].values.push_back (pc.channels[j].values[i]);
        }
      }
      return (result);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Publishes a visualization marker for a point.
      * @param p
      */
    void 
      showHandleMarker (geometry_msgs::PointStamped p)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = p.header.frame_id;
      marker.header.stamp = ros::Time();
      marker.ns = "handle_detector_vision";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = p.point.x;
      marker.pose.position.y = p.point.y;
      marker.pose.position.z = p.point.z;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
      marker.color.g = 1.0;
      marker.color.a = 1.0;
      marker_pub_.publish(marker);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    CvPoint 
      getDisparityCenter (CvRect& r)
    {
      CvMoments moments;
      double M00, M01, M10;

      cvSetImageROI(disp_, r);
      cvSetImageCOI(disp_, 1);
    	cvMoments(disp_,&moments,1);
    	M00 = cvGetSpatialMoment(&moments,0,0);
    	M10 = cvGetSpatialMoment(&moments,1,0);
    	M01 = cvGetSpatialMoment(&moments,0,1);
      cvResetImageROI(disp_);
      cvSetImageCOI(disp_, 0);

      CvPoint center;
      center.x = r.x+(int)(M10/M00);
      center.y = r.y+(int)(M01/M00);

      return (center);
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void 
      tryShrinkROI (CvRect& r)
    {
      cvSetImageROI(disp_, r);
    	IplImage* integral_patch = cvCreateImage(cvSize(r.width+1, r.height+1), IPL_DEPTH_32S, 1);
    	cvIntegral(disp_, integral_patch);
    	IndexedIplImage<int> ipatch(integral_patch);

    	CvRect r2 = r;
    	CvRect p;
    	p.x = 0;
    	p.y = 0;
    	p.height = r.height;
    	p.width = 1;
    	while (ipatch.integral_sum(p)==0 && p.x<r.width)
    		p.x++;
    	r2.x = r.x+p.x;

    	p.x = r.width-1;
    	while (ipatch.integral_sum(p)==0 && p.x>0)
    		p.x--;
    	r2.width = r.x-r2.x+p.x;

    	p.x = 0;
    	p.y = 0;
    	p.height = 1;
    	p.width = r.width;
    	while (ipatch.integral_sum(p)==0 && p.y<r.height)
    		p.y++;
    	r2.y = r.y+p.y;

    	p.y = r.height-1;
    	while (ipatch.integral_sum(p)==0 && p.y>0)
    		p.y--;
    	r2.height = r.y-r2.y+p.y;

    	r = r2;
      cvResetImageROI(disp_);
      cvReleaseImage(&integral_patch);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * \brief Determine if it's possible for handle to be in a specific ROI
     *
     * It looks at things like, height, approximate size of ROI, how flat it is.
     *
     * @param r
     * @return
     */
    bool 
      handlePossibleHere (CvRect &r)
    {
    	tryShrinkROI (r);

    	if (r.width < 10 || r.height < 10) 
      {
        ROS_INFO ("[handle_detector_vision] nhandlePossibleHere r.width %d r.height %d",r.width,r.height);
    		return (false);
    	}

    	cvSetImageROI (disp_, r);
    	cvSetImageCOI (disp_, 1);
    	int cnt;
    	const float nz_fraction = 0.1;
    	cnt = cvCountNonZero (disp_);
    	if (cnt < nz_fraction * r.width * r.height)
      {
        ROS_INFO ("[handle_detector_vision] cnt %d nz_fraction %f w %d h %d", cnt, nz_fraction, r.width, r.height);
    		cvResetImageROI (disp_);
    		cvSetImageCOI (disp_, 0);
    		return (false);
    	}
    	cvResetImageROI (disp_);
    	cvSetImageCOI (disp_, 0);


    	// compute least-squares handle plane
    	sensor_msgs::PointCloud pc = filterPointCloud (*cloud_, r);
    	CvScalar plane = estimatePlaneLS (pc);

    	cnt = 0;
    	double avg = 0;
    	double max_dist = 0;
    	for(size_t i = 0; i < pc.points.size (); ++i)
      {
    		geometry_msgs::Point32 p = pc.points[i];
    		double dist = fabs(plane.val[0] * p.x + plane.val[1] * p.y + plane.val[2] * p.z + plane.val[3]);
    		max_dist = max(max_dist, dist);
    		avg += dist;
    		cnt++;
    	}
    	avg /= cnt;
    	if(max_dist > 0.1 || avg < 0.001)
      {
        ROS_INFO("[handle_detector_vision] max_dist %f avg %f",max_dist,avg);
    		ROS_DEBUG("Not enough depth variation for handle candidate: %f, %f\n", max_dist, avg);
    		return false;
    	}

    	Stats xstats, ystats, zstats;
    	pointCloudStatistics (pc, xstats, ystats, zstats);

      double dx, dy;
    	dx = xstats.max - xstats.min;
    	dy = ystats.max - ystats.min;
    	
      if (dx > 0.25 || dy > 0.15)
      {
        ROS_INFO ("[handle_detector_vision] dx %f dy %f",dx,dy);
    		ROS_DEBUG ("Too big, discarding");
    		return (false);
    	}

    	geometry_msgs::PointStamped pin, pout;
    	pin.header.frame_id = cloud_->header.frame_id;
    	pin.header.stamp = cloud_->header.stamp;
    	pin.point.x = xstats.mean;
    	pin.point.y = ystats.mean;
    	pin.point.z = zstats.mean;
      if (!tf_.waitForTransform ("base_footprint", pin.header.frame_id, pin.header.stamp, ros::Duration(5.0)))
      {
        ROS_INFO ("Cannot transform from base_footprint to %s", pin.header.frame_id.c_str());
        return (false);
      }
      tf_.transformPoint ("base_footprint", pin, pout);
    	if (pout.point.z > max_height_ || pout.point.z < min_height_)
      {
    		ROS_INFO("Height not within admissable range: %f\n", pout.point.z);
    		return (false);
    	}

    	ROS_DEBUG ("Handle at: (%d,%d,%d,%d)", r.x,r.y,r.width, r.height);

    	return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * \brief Start handle detection
     */
    void 
      findHandleCascade (vector<CvRect> & handle_rect)
    {
      IplImage *gray = cvCreateImage (cvSize (left_->width, left_->height), 8, 1);
      cvCvtColor (left_, gray, CV_BGR2GRAY);
      cvEqualizeHist (gray, gray);
      cvClearMemStorage (storage);

      if (cascade)
      {
        CvSeq *handles = cvHaarDetectObjects (gray, cascade, storage, 1.1, 2, 0, //|CV_HAAR_FIND_BIGGEST_OBJECT
        //|CV_HAAR_DO_ROUGH_SEARCH
        //|CV_HAAR_DO_CANNY_PRUNING
        //|CV_HAAR_SCALE_IMAGE
        cvSize (10, 10));

        for (int i = 0;i < (handles ? handles->total : 0);i++)
        {
          CvRect *r = (CvRect*)cvGetSeqElem(handles, i);

          if (handlePossibleHere (*r))
          {
            handle_rect.push_back (*r);
            if (display_)
            {
              cvRectangle (left_, cvPoint(r->x, r->y), cvPoint(r->x + r->width, r->y + r->height), CV_RGB(255, 255, 0));
              cvRectangle (disp_, cvPoint(r->x, r->y), cvPoint(r->x + r->width, r->y + r->height), CV_RGB(255, 255, 255));
            }
          }
          else
          {
            if (display_)
              cvRectangle (left_, cvPoint(r->x, r->y), cvPoint(r->x + r->width, r->y + r->height), CV_RGB(255, 0, 0));
           }
        }
      }
      else
        ROS_ERROR ("Cannot look for handle, no detector loaded");
      cvReleaseImage (&gray);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * \brief Checks if a point should belong to a cluster
     *
     * @param center
     * @param p
     * @return
     */
    inline bool 
      belongsToCluster (pair<float,float> center, pair<float,float> p)
    {
    	return (fabs (center.first-p.first) < 3 && fabs (center.second-p.second) < 3);
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * Helper function.
     *
     * @param r
     * @return
     */
    inline pair<float,float> 
      rectCenter (const CvRect& r)
    {
    	return (make_pair (r.x+(float)r.width/2,r.y+(float)r.height/2));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * \brief Decide the handle position from several detections across multiple frames
     *
     * This method does some simple clustering of all the bounding boxes and picks the cluster with
     * the most elements.
     *
     * @param handle_rect The handle bounding boxes
     * @param handle Point indicating the real-world handle position
     * @return
     */
    bool 
      decideHandlePosition (vector<CvRect>& handle_rect, geometry_msgs::PointStamped & handle)
    {
    	if (handle_rect.size() == 0)
      {
        ROS_WARN ("Handle rect size is 0!");
    		return (false);
      }

    	vector<int> cluster_size;
    	vector<pair<float,float> > centers;
    	vector<pair<float,float> > sizes;

    	for (size_t i = 0;i < handle_rect.size (); ++i)
      {
    		CvRect r = handle_rect[i];
    		pair<float,float> crt = rectCenter (r);
    		bool found_cluster = false;
    		for (size_t j = 0;j < centers.size (); ++j)
        {
    			if (belongsToCluster (centers[j], crt))
          {
    				int n = cluster_size[j];
    				centers[j].first = (centers[j].first*n+crt.first)/(n+1);
    				centers[j].second = (centers[j].second*n+crt.second)/(n+1);
    				sizes[j].first = (sizes[j].first*n+r.width)/(n+1);
    				sizes[j].second = (sizes[j].second*n+r.height)/(n+1);
    				cluster_size[j]++;
    				found_cluster = true;
    				break;
    			}
    		}
    		if (!found_cluster)
        {
    			centers.push_back (crt);
    			sizes.push_back (make_pair (r.width,r.height));
    			cluster_size.push_back(1);
    		}
    	}

    	int max_ind = 0;
    	int max = cluster_size[0];

    	for (size_t i = 0;i < cluster_size.size (); ++i)
      {
    		if (cluster_size[i] > max)
        {
    			max = cluster_size[i];
    			max_ind = i;
    		}
    	}

    	CvRect bbox;
    	bbox.x = (int) centers[max_ind].first - (int)(sizes[max_ind].first/2);
    	bbox.y = (int) centers[max_ind].second - (int)(sizes[max_ind].second/2);
    	bbox.width = (int) sizes[max_ind].first;
    	bbox.height = (int) sizes[max_ind].second;

    	sensor_msgs::PointCloud handle_cloud = filterPointCloud (*cloud_, bbox);

    	Stats xstats;
    	Stats ystats;
    	Stats zstats;
    	pointCloudStatistics (handle_cloud, xstats, ystats, zstats );

      geometry_msgs::PointStamped handle_stereo;

      handle_stereo.header.frame_id = cloud_->header.frame_id;
      handle_stereo.header.stamp = cloud_->header.stamp;
      handle_stereo.point.x = xstats.mean;
      handle_stereo.point.y = ystats.mean;
      handle_stereo.point.z = zstats.mean;

      if (!tf_.waitForTransform (handle.header.frame_id, handle_stereo.header.frame_id,
                                 handle_stereo.header.stamp, ros::Duration(5.0)))
      {
        ROS_ERROR ("Cannot transform from %s to %s", handle.header.frame_id.c_str(),
                   handle_stereo.header.frame_id.c_str());
        return (false);
      }
      tf_.transformPoint (handle.header.frame_id, handle_stereo, handle);

      ROS_INFO ("Clustered Handle at: (%d,%d,%d,%d)", bbox.x,bbox.y,bbox.width, bbox.height);


      if (display_)
      {
        cvRectangle (left_, cvPoint(bbox.x, bbox.y), cvPoint(bbox.x + bbox.width, bbox.y + bbox.height), CV_RGB(0, 255, 0));
        cvShowImage ("left", left_);
      }
      showHandleMarker (handle_stereo);

      return (true);
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * \brief Runs the handle detector
     *
     * @param handle Position of detected handle
     * @return True if handle was found, false otherwise.
     */
    bool 
      runHandleDetector (geometry_msgs::PointStamped & handle)
    {
    	vector<CvRect> handle_rect;

        // acquire cv_mutex lock
    	boost::unique_lock<boost::mutex> lock (data_lock_);

      for (int i = 0; i < frames_no_; ++i) 
      {
        ROS_INFO ("DoorHandleDetector: Waiting for stereo data");
        got_images_ = false;
        preempted_ = false;
        // block until images are available to process
        start_image_wait_ = ros::Time::now ();
        while (!got_images_ && !preempted_)
          data_cv_.wait (lock);
        if (preempted_) 
        {
          ROS_INFO ("DoorHandleDetector: detect loop preempted, stereo data not received");
          return (false);
        }

        if (display_)
          // show original disparity
          cvShowImage ("disparity_original", disp_);
        ROS_INFO ("DoorHandleDetector: Running handle detection");
        
        // eliminate from disparity locations that cannot contain a handle
        applyPositionPrior ();
        
        // run cascade classifier
        findHandleCascade (handle_rect);
        
        if(display_)
        {
          // show filtered disparity
          cvShowImage ("disparity", disp_);
          // show left image
          cvShowImage ("left", left_);
          cvWaitKey (100);
        }
      }

      bool found = decideHandlePosition (handle_rect, handle);
      return (found);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * \brief Service call to detect doors
     */
    bool 
      detectHandleSrv (door_handle_detector::DoorsDetector::Request & req, door_handle_detector::DoorsDetector::Response & resp)
    {
    	ROS_INFO_STREAM ("detectHandleSrv thread id=" << boost::this_thread::get_id());
    	preempted_ = false;

    	ROS_INFO ("door_handle_detector_vision: Service called");
      geometry_msgs::PointStamped handle;
      handle.header.frame_id = req.door.header.frame_id;   // want handle in the same frame as the door
    	subscribeStereoData ();
      bool found = runHandleDetector (handle);
    	unsubscribeStereoData ();

      if (!found)
      {
        ROS_WARN ("No handles found.");
        return (false);
      }
      geometry_msgs::PointStamped handle_transformed;
      // transform the point in the expected frame
      if (!tf_.waitForTransform (req.door.header.frame_id, handle.header.frame_id, handle.header.stamp, ros::Duration (5.0)))
      {
        ROS_ERROR ("Cannot transform from %s to %s", handle.header.frame_id.c_str (), req.door.header.frame_id.c_str ());
        return (false);
      }
      
      tf_.transformPoint (req.door.header.frame_id, handle, handle_transformed);

      resp.doors.resize (1);
      if (found)
      {
        resp.doors[0] = req.door;
        resp.doors[0].header.stamp = handle.header.stamp; // set time stamp
        resp.doors[0].handle.x = handle.point.x;
        resp.doors[0].handle.y = handle.point.y;
        resp.doors[0].handle.z = handle.point.z;
        resp.doors[0].weight = 1;
      }
      else
      {
        resp.doors[0] = req.door;
        resp.doors[0].weight = -1;
      }
      return (true);
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool 
      preempt (std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
    {
    	ROS_INFO ("Preempting.");
    	preempted_ = true;
    	return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * \brief Computes a least-squares estimate of a plane.
     *
     * @param points The point cloud
     * @return Plane in Hessian normal form
     */
    CvScalar 
      estimatePlaneLS (sensor_msgs::PointCloud points)
    {
      CvScalar plane;
      int cnt = points.points.size();
      if (cnt==0)
      {
        plane.val[0] = plane.val[1] = plane.val[2] = plane.val[3] = -1;
        return plane;
      }
      CvMat *A = cvCreateMat(cnt, 3, CV_32FC1);
      for(int i = 0;i < cnt;++i)
      {
        geometry_msgs::Point32 p = points.points[i];
        cvmSet(A, i, 0, p.x);
        cvmSet(A, i, 1, p.y);
        cvmSet(A, i, 2, p.z);
      }
      vector<float> ones(cnt, 1);
      CvMat B;
      cvInitMatHeader(&B, cnt, 1, CV_32FC1, &ones[0]);
      CvMat *X = cvCreateMat(3, 1, CV_32FC1);
      int ok = cvSolve(A, &B, X, CV_SVD);
      if(ok)
      {
        float *xp = X->data.fl;
        float d = sqrt(xp[0] * xp[0] + xp[1] * xp[1] + xp[2] * xp[2]);
        plane.val[0] = xp[0] / d;
        plane.val[1] = xp[1] / d;
        plane.val[2] = xp[2] / d;
        plane.val[3] = -1 / d;
      }
      else
        plane.val[0] = plane.val[1] = plane.val[2] = plane.val[3] = -1;
      cvReleaseMat (&A);
      return (plane);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * \brief Filters cloud point, retains only regions that could contain a handle
     */
    void 
      applyPositionPrior ()
    {
      sensor_msgs::PointCloud base_cloud;
      if (!tf_.waitForTransform ("base_footprint", cloud_->header.frame_id, cloud_->header.stamp, ros::Duration (5.0)))
      {
        ROS_ERROR ("Cannot transform from base_footprint to %s", cloud_->header.frame_id.c_str ());
        return;
      }
      tf_.transformPointCloud ("base_footprint", *cloud_, base_cloud);
      
      int xchan = -1, ychan = -1;
      for (size_t i = 0; i < base_cloud.channels.size (); ++i)
      {
        if (base_cloud.channels[i].name == "u")
          xchan = i;
        if (base_cloud.channels[i].name == "v")
          ychan = i;
      }

      if (xchan == -1 || ychan == -1)
      {
        ROS_WARN ("I can't find image coordinates in the point cloud, no filtering done.");
        return;
      }

      unsigned char *pd = (unsigned char*)(disp_->imageData);
      int ws = disp_->widthStep;
      for (size_t i = 0; i < base_cloud.get_points_size (); ++i)
      {
        int x = (int)(base_cloud.channels[xchan].values[i]);
        int y = (int)(base_cloud.channels[ychan].values[i]);

       if (base_cloud.points[i].z > max_height_ || base_cloud.points[i].z < min_height_) 
       {
         // pointer to the current pixel
         unsigned char* crt_pd = pd + x * ws + y;
         *crt_pd = 0;
       }
      }
    }
  
  public:
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
    * Needed for OpenCV event loop, to show images
    * @return
    */
    bool
      spin ()
    {
      ros::Rate r (10);
      while (nh_.ok ())
      {
        data_lock_.lock ();
        if (!got_images_ )
        {
          if ((ros::Time::now () - start_image_wait_) > ros::Duration (timeout_))
          {
            preempted_ = true;
            data_cv_.notify_all ();
          }
        }
        data_lock_.unlock ();

        if (display_)
        {
          int key = cvWaitKey (10) & 0x00FF;
          if (key == 27) //ESC
            nh_.shutdown ();
        }
        else
          r.sleep ();
        ros::spinOnce ();
      }

      return (true);
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void 
      serviceThread ()
    {
  //		ROS_INFO_STREAM("Starting thread " << boost::this_thread::get_id());
      ros::NodeHandle n;

      ros::Rate r (10);
      while (n.ok ())
      {
        service_queue_.callAvailable ();
        r.sleep ();
      }
    }
};

int 
  main (int argc, char **argv)
{
	for (int i = 0; i<argc; ++i)
		cout << "(" << i << "): " << argv[i] << endl;

	ros::init (argc, argv, "handle_detector_vision");
	HandleDetector detector;
	detector.spin ();

	return (0);
}
