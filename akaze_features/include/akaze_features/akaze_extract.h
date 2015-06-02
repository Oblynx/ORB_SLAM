#ifndef AKAZE_FEATURES_AKAZE_EXTRACT_H
#define AKAZE_FEATURES_AKAZE_EXTRACT_H

#include "akaze_features/AKAZE.h"
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

class AkazeFeatureExtractor
{
  public:

    explicit AkazeFeatureExtractor();
    ~AkazeFeatureExtractor()
    {
      delete featureDetectorPtr_;
    }
    void imgCallback(const sensor_msgs::ImageConstPtr& imgMsg);
  private:

    ros::NodeHandle nh_;

    akaze_features::AKAZE* featureDetectorPtr_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber imgSubscriber_;
};

#endif // AKAZE_FEATURES_AKAZE_EXTRACT_H 
