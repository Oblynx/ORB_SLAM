/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
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
*   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
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
*
* Author: Konstantinos Samaras-Tsakiris
*********************************************************************/

#ifndef ORB_SLAM_WRAPPER_H
#define ORB_SLAM_WRAPPER_H

#include <fstream>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>

#include "opencv2/core/core.hpp"

#include "Tracking.h"
#include "FramePublisher.h"
#include "Map.h"
#include "MapPublisher.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"

#include "Converter.h"

namespace ORB_SLAM{

/**
 @class Wrapper
 @brief Wraps the "main.cc" function that initializes orb_slam, in order to inte-
 		grate it into a larger ROS project with a state_manager
 **/
class Wrapper{
	public:
		/**
		@brief Constructor. Gets file paths from ROS parameter server
		@param ns: Nodehandle namespace
		**/
		Wrapper(const std::string& ns);
		void start();
		void stop();
	private:
		void mainLoop();

		ros::NodeHandle nh_;
		std::string paths[2];

		//! orbslam related fields
		//! Path to settings file
		string strSettingsFile_;
		//! Path to vocabulary file
		string strVocFile_;
		ORB_SLAM::ORBVocabulary vocabulary_;

		// Create the map
  	ORB_SLAM::Map world_;
  	// Create Frame Publisher for image_view
  	ORB_SLAM::FramePublisher framePub_;
  	// Create Map Publisher for Rviz
  	ORB_SLAM::MapPublisher mapPub_;

  	// Create tracker
  	ORB_SLAM::Tracking* pTracker_;
  	boost::thread trackingThread_;
  	// Create KeyFrame Database
  	ORB_SLAM::KeyFrameDatabase* database_;
  	// Initialize the Local Mapping Thread
  	ORB_SLAM::LocalMapping* localMapper_;
  	boost::thread localMappingThread_;
  	// Initialize the Loop Closing Thread
  	ORB_SLAM::LoopClosing* loopCloser_;
  	boost::thread loopClosingThread_;
  	
  	boost::thread mainLoopThread_;

  	float fps_;
};

}	// namespace ORB_SLAM

#endif //ORB_SLAM_WRAPPER_H
