#include "akaze_features/akaze_extract.h"


AkazeFeatureExtractor::AkazeFeatureExtractor(): it_(nh_)
{
  ROS_INFO("Creating new Akaze Feature Extractor!");
  akaze_features::AKAZEOptions options;
  options.img_width = 640;
  options.img_height = 480;

  featureDetectorPtr_ = new akaze_features::AKAZE(options);

  if (featureDetectorPtr_ == NULL)
  {
    ROS_ERROR("Could not create AKAZE feature extractor!");
    ros::shutdown();
  }

  imgSubscriber_ = it_.subscribe("/camera/image_raw", 1, 
      &AkazeFeatureExtractor::imgCallback, this);
  ROS_INFO("Create Akaze features extractor object!");
}



void AkazeFeatureExtractor::imgCallback(
    const sensor_msgs::ImageConstPtr& imgMsg)
{
  ROS_INFO("Received new image!");
  cv_bridge::CvImagePtr imgPtr;
  try
  {
    imgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  std::vector<cv::KeyPoint> akazeKeypoints;
  cv::Mat akazeDescriptors;

  cv::Mat featureImg;
  cv::cvtColor(imgPtr->image, featureImg, CV_BGR2GRAY);

  struct timeval startwtime, endwtime; //<! Structs used for calculating 

  featureImg.convertTo(featureImg, CV_32F, 1.0f/255.0f, 0);


  gettimeofday(&startwtime , NULL);
  featureDetectorPtr_->Create_Nonlinear_Scale_Space(featureImg);
  featureDetectorPtr_->Feature_Detection(akazeKeypoints);

  featureDetectorPtr_->Compute_Descriptors(akazeKeypoints, akazeDescriptors);

  gettimeofday(&endwtime , NULL);
  double featuresTime = static_cast<double>((endwtime.tv_usec - 
        startwtime.tv_usec) / 1.0e6 
      + endwtime.tv_sec - startwtime.tv_sec);
  ROS_INFO_STREAM("Calculation time for the" 
      << " features in the frame is " << featuresTime);

  // cv::drawKeypoints(imgPtr->image, akazeKeypoints, imgPtr->image);
  akaze_features::draw_keypoints(imgPtr->image, akazeKeypoints);

  cv::imshow("Akaze Keypoints", imgPtr->image);
  int key = 255 & cv::waitKey(5);
  if (key == 27)
  {
    ROS_INFO("Shutting down the node!");
    ros::shutdown();
  }
}


