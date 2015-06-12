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

#include "orb_slam/wrapper.h"

namespace ORB_SLAM{

Wrapper::Wrapper(const std::string& ns):
		nh_(ns), world_(), framePub_(), mapPub_(&world_)
{
	ROS_INFO_STREAM(endl << "ORB-SLAM Copyright (C) 2014 Raul Mur-Artal" << endl <<
            "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
            "This is free software, and you are welcome to redistribute it" << endl <<
            "under certain conditions. See LICENSE.txt." << endl);

  // Load Settings and Check
  nh_.param<std::string>("ORB_vocabulary_path", paths[0], "Data/ORBvoc.yml");
  nh_.param<std::string>("orb_slam_settings_path", paths[1], "Data/default_settings.yaml");

  strSettingsFile_ = ros::package::getPath("orb_slam")+"/"+paths[1].c_str();

  cv::FileStorage fsSettings(strSettingsFile_.c_str(), cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
    ROS_FATAL("[%s]: Wrong path to settings. Path must be absolute or relative\
              to ORB_SLAM package directory. Path given:\n%s",
              nh_.getNamespace().c_str(), strSettingsFile_.c_str());
    ros::shutdown();
  }

  //Load ORB Vocabulary
  strVocFile_ = ros::package::getPath("orb_slam")+"/"+paths[0].c_str();
  ROS_INFO("[%s]: Loading ORB Vocabulary. This could take a while.\n", nh_.getNamespace().c_str());
  cv::FileStorage fsVoc(strVocFile_.c_str(), cv::FileStorage::READ);
  if(!fsVoc.isOpened())
  {
    ROS_FATAL("[%s]: Wrong path to vocabulary. Path must be absolute or relative\
              to ORB_SLAM package directory. Path given:\n%s",
              nh_.getNamespace().c_str(), strVocFile_.c_str());
    ros::shutdown();
  }

  vocabulary_.load(fsVoc);
  ROS_INFO("[%s]: Vocabulary loaded!\n\n", nh_.getNamespace().c_str());

  //This "main" thread will show the current processed frame and publish the map
  fps_ = fsSettings["Camera.fps"];
  if(fps_ == 0)
      fps_= 30;

  database_= new ORB_SLAM::KeyFrameDatabase(vocabulary_);
  framePub_.SetMap(&world_);
  //Initialize the Tracking, LocalMapping & LoopClosing threads
  pTracker_= new ORB_SLAM::Tracking(&vocabulary_, &framePub_, &mapPub_, &world_,
                                    strSettingsFile_);
  localMapper_= new ORB_SLAM::LocalMapping(&world_);
  loopCloser_= new ORB_SLAM::LoopClosing(&world_, database_, &vocabulary_);

  ROS_INFO("[orb_slam]: Successfully constructed.");
}

void Wrapper::start(){
  //Launch the Tracking, LocalMapping & LoopClosing threads
  trackingThread_= boost::thread(&ORB_SLAM::Tracking::Run, pTracker_);
  pTracker_->SetKeyFrameDatabase(database_);
  localMappingThread_= boost::thread(&ORB_SLAM::LocalMapping::Run, localMapper_);
  loopClosingThread_= boost::thread(&ORB_SLAM::LoopClosing::Run, loopCloser_);

  //Set pointers between threads
  pTracker_->SetLocalMapper(localMapper_);
  pTracker_->SetLoopClosing(loopCloser_);
  localMapper_->SetTracker(pTracker_);
  localMapper_->SetLoopCloser(loopCloser_);
  loopCloser_->SetTracker(pTracker_);
  loopCloser_->SetLocalMapper(localMapper_);

  mainLoopThread_= boost::thread(&ORB_SLAM::Wrapper::mainLoop, this);

  ROS_INFO("[orb_slam]: Threads launched, starting work.");
}

void Wrapper::stop(){
	// Save keyframe poses at the end of the execution
  ofstream f;

  vector<ORB_SLAM::KeyFrame*> vpKFs = world_.GetAllKeyFrames();
  sort(vpKFs.begin(),vpKFs.end(),ORB_SLAM::KeyFrame::lId);

  ROS_INFO("[%s]: Saving Keyframe Trajectory to KeyFrameTrajectory.txt",
           nh_.getNamespace().c_str());
  string strFile = ros::package::getPath("orb_slam")+"/"+"KeyFrameTrajectory.txt";
  f.open(strFile.c_str());
  f << fixed;

  for(size_t i=0; i<vpKFs.size(); i++)
  {
      ORB_SLAM::KeyFrame* pKF = vpKFs[i];

      if(pKF->isBad())
          continue;

      cv::Mat R = pKF->GetRotation().t();
      vector<float> q = ORB_SLAM::Converter::toQuaternion(R);
      cv::Mat t = pKF->GetCameraCenter();
      f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " 
      	<< t.at<float>(0)<< " " << t.at<float>(1) << " " << t.at<float>(2)
        << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

  }
  f.close();
}

void Wrapper::mainLoop(){
  ros::Rate r(fps_);
  
  while (ros::ok())
  {
      framePub_.Refresh();
      mapPub_.Refresh();
      pTracker_->CheckResetByPublishers();
      r.sleep();
  }
}

}	// namespace orb_slam
