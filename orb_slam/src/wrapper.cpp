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

#include "wrapper.h"

namespace orb_slam{

Wrapper::Wrapper(int argc, char** argv, const std::string& ns) : 
								argc_(argc), argv_(argv), nh_(ns) {
	ROS_INFO_STREAM(<< endl << "ORB-SLAM Copyright (C) 2014 Raul Mur-Artal" << endl <<
            "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
            "This is free software, and you are welcome to redistribute it" << endl <<
            "under certain conditions. See LICENSE.txt." << endl);
	if(argc != 3)
  {
      ROS_ERROR("[%s]: Insufficient input arguments\n", nh_.getNamespace());
      ros::shutdown();
      return 1;
  }

  // Load Settings and Check
  string strSettingsFile = ros::package::getPath("orb_slam")+"/"+argv[2];

  cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
      ROS_ERROR("[%s]: Wrong path to settings. Path must be absolute or relative
      					 to ORB_SLAM package directory.", nh_.getNamespace());
      ros::shutdown();
      return 1;
  }

  //Create Frame Publisher for image_view
  ORB_SLAM::FramePublisher FramePub;

  //Load ORB Vocabulary
  string strVocFile = ros::package::getPath("orb_slam")+"/"+argv[1];
  ROS_INFO("[%s]: Loading ORB Vocabulary. This could take a while.\n", nh_.getNamespace());
  cv::FileStorage fsVoc(strVocFile.c_str(), cv::FileStorage::READ);
  if(!fsVoc.isOpened())
  {
      ROS_ERROR("[%s]: Wrong path to vocabulary. Path must be absolute or relative
      					 to ORB_SLAM package directory.", nh_.getNamespace());
      ros::shutdown();
      return 1;
  }
  ORB_SLAM::ORBVocabulary Vocabulary;
  Vocabulary.load(fsVoc);

  ROS_INFO("[%s]: Vocabulary loaded!\n\n", nh_.getNamespace());

  //Create KeyFrame Database
  ORB_SLAM::KeyFrameDatabase Database(Vocabulary);

  //Create the map
  ORB_SLAM::Map World;

  FramePub.SetMap(&World);

  //Create Map Publisher for Rviz
  ORB_SLAM::MapPublisher MapPub(&World);



  //Initialize the Tracking Thread and launch
  ORB_SLAM::Tracking Tracker(&Vocabulary, &FramePub, &MapPub, &World, strSettingsFile);
  boost::thread trackingThread(&ORB_SLAM::Tracking::Run,&Tracker);

  Tracker.SetKeyFrameDatabase(&Database);

  //Initialize the Local Mapping Thread and launch
  ORB_SLAM::LocalMapping LocalMapper(&World);
  boost::thread localMappingThread(&ORB_SLAM::LocalMapping::Run,&LocalMapper);

  //Initialize the Loop Closing Thread and launch
  ORB_SLAM::LoopClosing LoopCloser(&World, &Database, &Vocabulary);
  boost::thread loopClosingThread(&ORB_SLAM::LoopClosing::Run, &LoopCloser);

  //Set pointers between threads
  Tracker.SetLocalMapper(&LocalMapper);
  Tracker.SetLoopClosing(&LoopCloser);

  LocalMapper.SetTracker(&Tracker);
  LocalMapper.SetLoopCloser(&LoopCloser);

  LoopCloser.SetTracker(&Tracker);
  LoopCloser.SetLocalMapper(&LocalMapper);

  //This "main" thread will show the current processed frame and publish the map
  float fps = fsSettings["Camera.fps"];
  if(fps==0)
      fps=30;

}

Wrapper::start(){
	ros::Rate r(fps);


  while (ros::ok())
  {
      FramePub.Refresh();
      MapPub.Refresh();
      Tracker.CheckResetByPublishers();
      r.sleep();
  }
}

Wrapper::stop(){
	// Save keyframe poses at the end of the execution
  ofstream f;

  vector<ORB_SLAM::KeyFrame*> vpKFs = World.GetAllKeyFrames();
  sort(vpKFs.begin(),vpKFs.end(),ORB_SLAM::KeyFrame::lId);

  ROS_INFO("[%s]: Saving Keyframe Trajectory to KeyFrameTrajectory.txt", nh_.getNamespace());
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
      f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
        << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

  }
  f.close();
  ros::shutdown();
}

}	// namespace orb_slam
