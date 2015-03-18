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

/*! \mainpage
 *  \htmlinclude manifest.html
 */

#ifndef TOPIC_SYNCHRONIZER2_HH
#define TOPIC_SYNCHRONIZER2_HH

#include "ros/ros.h"
#include <boost/thread/mutex.hpp>

  //! A class for synchronizing incoming topics
  /*!
   * The Topic Synchronizerc ismpassed a function pointer at construction to be called every time
   * all of your topics have arrived.
   *
   * Example of usage:
   *
   * TopicSynchronizer sync(&MyClass::syncCallback, this);
   *
   * ros::Subscriber left_image_sub_ = nh_.subscribe("stereo/left/image_rect", 1, sync.synchronize(&MyClass::leftImageCallback, this));
	 ros::Subscriber right_image_sub_ = nh_.subscribe("stereo/right/image_rect", 1, sync.synchronize(&MyClass::rightImageCallback, this));
	 ros::Subscriber disparity_sub_ = nh_.subscribe("stereo/disparity", 1, sync.synchronize(&MyClass::disparityImageCallback, this));
	 ros::Subscriber cloud_sub_ = nh_.subscribe("stereo/cloud", 1, sync.synchronize(&MyClass::cloudCallback, this));
   *
   *
   */
class TopicSynchronizer
{
	int expected_count_;
	int count_;
	ros::Time time_;
	boost::function<void ()> callback_;
	boost::mutex mutex_;

public:

	template <typename T>
	TopicSynchronizer(void(T::*fp)(), T* obj) : callback_(boost::bind(fp,obj))
	{
		reset();
	}

	TopicSynchronizer(void (*fp)()) : callback_(fp)
	{
		reset();
	}


	template <typename T, typename M>
	class CallbackFunctor {
		TopicSynchronizer* synchronizer_;
		const boost::function<void (const boost::shared_ptr<M const>&)> callback_;

	public:

		CallbackFunctor(TopicSynchronizer* synchronizer, const boost::function<void (const boost::shared_ptr<M const>&)>& callback) :
			synchronizer_(synchronizer), callback_(callback)
		{
		}

		void operator()(const boost::shared_ptr<M const>& message)
		{
			callback_(message);
			synchronizer_->update(message->header.stamp);
		}
	};

	template <typename T, typename M>
	const boost::function<void (const boost::shared_ptr<M const>&)> synchronize(void(T::*fp)(const boost::shared_ptr<M const>&), T* obj)
	{
	  boost::mutex::scoped_lock lock(mutex_);

		expected_count_++;
		return CallbackFunctor<T,M>(this, boost::bind(fp, obj, _1));
	}

	template <typename T, typename M>
	const boost::function<void (const boost::shared_ptr<M const>&)> synchronize(void(T::*fp)(const boost::shared_ptr<M const>&), const boost::shared_ptr<T>& obj)
	{
	  boost::mutex::scoped_lock lock(mutex_);

		expected_count_++;
		return CallbackFunctor<T,M>(this, boost::bind(fp, obj.get(), _1));
	}

	template <typename T, typename M>
	const boost::function<void (const boost::shared_ptr<M const>&)> synchronize(void(*fp)(const boost::shared_ptr<M const>&))
	{
	  boost::mutex::scoped_lock lock(mutex_);

		expected_count_++;
		return CallbackFunctor<T,M>(this, boost::function<void(const boost::shared_ptr<M>&)>(fp));
	}

	template <typename T, typename M>
	const boost::function<void (const boost::shared_ptr<M const>&)> synchronize(const boost::function<void (const boost::shared_ptr<M const>&)>& callback)
	{
	  boost::mutex::scoped_lock lock(mutex_);
		expected_count_++;
		return CallbackFunctor<T,M>(this, callback);
	}

	void reset()
	{
	  boost::mutex::scoped_lock lock(mutex_);
		expected_count_ = 0;
	}

	void update(const ros::Time& time)
	{
	  boost::mutex::scoped_lock lock(mutex_);
		if (count_==0 || time>time_) {
			time_ = time;
			count_ = 0;
		}

		if (time==time_) {
			count_++;
		}

		if (count_==expected_count_) {
			callback_();
		}
	}
};

#endif
