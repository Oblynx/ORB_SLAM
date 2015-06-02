#include "akaze_features/akaze_extract.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "akaze_feature_extractor");
  AkazeFeatureExtractor extractor;
  ros::spin();
  return 0;
}
