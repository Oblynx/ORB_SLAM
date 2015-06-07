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

Wrapper::Wrapper(int argc, char** argv, const std::string& ns):
		argc_(argc), argv_(argv), nh_(ns), world_(), framePub_(), mapPub_(&world_)
	{
	ROS_INFO_STREAM(endl << "ORB-SLAM Copyright (C) 2014 Raul Mur-Artal" << endl <<
            "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
            "This is free software, and you are welcome to redistribute it" << endl <<
            "under certain conditions. See LICENSE.txt." << endl);
	if(argc != 3)
  {
      ROS_FATAL("[%s]: Insufficient input arguments\n", nh_.getNamespace().c_str());
  }

  // Load Settings and Check
  string strSettingsFile = ros::package::getPath("orb_slam")+"/"+argv[2];

  cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
      ROS_FATAL("[%s]: Wrong path to settings. Path must be absolute or relative to ORB_SLAM package directory.",
                nh_.getNamespace().c_str());
  }

  

  //Load ORB Vocabulary
  string strVocFile = ros::package::getPath("orb_slam")+"/"+argv[1];
  ROS_INFO("[%s]: Loading ORB Vocabulary. This could take a while.\n", nh_.getNamespace().c_str());
  cv::FileStorage fsVoc(strVocFile.c_str(), cv::FileStorage::READ);
  if(!fsVoc.isOpened())
  {
      ROS_FATAL("[%s]: Wrong path to vocabulary. Path must be absolute or relative to ORB_SLAM package directory.",
                nh_.getNamespace().c_str());
  }
  ORB_SLAM::ORBVocabulary Vocabulary;
  Vocabulary.load(fsVoc);

  ROS_INFO("[%s]: Vocabulary loaded!\n\n", nh_.getNamespace().c_str());

  //Create KeyFrame Database
  ORB_SLAM::KeyFrameDatabase Database(Vocabulary);

  
  framePub_.SetMap(&world_);

  pTracker_= new ORB_SLAM::Tracking(&Vocabulary, &framePub_, &mapPub_, &world_,
                                   strSettingsFile);
  boost::thread trackingThread(&ORB_SLAM::Tracking::Run, pTracker_);

  pTracker_->SetKeyFrameDatabase(&Database);

  //Initialize the Local Mapping Thread and launch
  ORB_SLAM::LocalMapping LocalMapper(&world_);
  boost::thread localMappingThread(&ORB_SLAM::LocalMapping::Run,&LocalMapper);

  //Initialize the Loop Closing Thread and launch
  ORB_SLAM::LoopClosing LoopCloser(&world_, &Database, &Vocabulary);
  boost::thread loopClosingThread(&ORB_SLAM::LoopClosing::Run, &LoopCloser);

  //Set pointers between threads
  pTracker_->SetLocalMapper(&LocalMapper);
  pTracker_->SetLoopClosing(&LoopCloser);

  LocalMapper.SetTracker(pTracker_);
  LocalMapper.SetLoopCloser(&LoopCloser);

  LoopCloser.SetTracker(pTracker_);
  LoopCloser.SetLocalMapper(&LocalMapper);

  fps_ = fsSettings["Camera.fps"];
  if(fps_==0)
      fps_=30;

}

void Wrapper::start(){
	ros::Rate r(fps_);
  while (ros::ok())
  {
      framePub_.Refresh();
      mapPub_.Refresh();
      pTracker_->CheckResetByPublishers();
      r.sleep();
  }
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

}	// namespace orb_slam
