//=============================================================================
//
// akaze_match.cpp
// Authors: Pablo F. Alcantarilla (1), Jesus Nuevo (2)
// Institutions: Toshiba Research Europe Ltd (1)
//               TrueVision Solutions (2)
// Date: 07/10/2014
// Email: pablofdezalc@gmail.com
//
// AKAZE Features Copyright 2014, Pablo F. Alcantarilla, Jesus Nuevo
// All Rights Reserved
// See LICENSE for the license information
//=============================================================================

/**
 * @file akaze_match.cpp
 * @brief Main program for matching two images with AKAZE features
 * @date Oct 07, 2014
 * @author Pablo F. Alcantarilla
 */

#include "akaze_features/AKAZE.h"


/* ************************************************************************* */
// Image matching options
const float MIN_H_ERROR = 2.50f;            ///< Maximum error in pixels to accept an inlier
const float DRATIO = 0.80f;                 ///< NNDR Matching value

/* ************************************************************************* */
/**
 * @brief This function parses the command line arguments for setting A-KAZE parameters
 * and image matching between two input images
 * @param options Structure that contains A-KAZE settings
 * @param img_path1 Path for the first input image
 * @param img_path2 Path for the second input image
 * @param homography_path Path for the file that contains the ground truth homography
 */
int parse_input_options(akaze_features::AKAZEOptions &options,
                        std::string& img_path1, std::string& img_path2,
                        std::string& homography_path,
                        int argc, char *argv[]);

/* ************************************************************************* */
int main(int argc, char *argv[]) {

  // Variables
  akaze_features::AKAZEOptions options;
  cv::Mat img1, img1_32, img2, img2_32, img1_rgb, img2_rgb, img_com, img_r;
  std::string img_path1, img_path2, homography_path;
  float ratio = 0.0, rfactor = .60;
  int nkpts1 = 0, nkpts2 = 0, nmatches = 0, ninliers = 0, noutliers = 0;

  std::vector<cv::KeyPoint> kpts1, kpts2;
  std::vector<std::vector<cv::DMatch> > dmatches;
  cv::Mat desc1, desc2;
  cv::Mat HG;

  // Variables for measuring computation times
  double t1 = 0.0, t2 = 0.0;
  double takaze = 0.0, tmatch = 0.0;

  // Parse the input command line options
  if (parse_input_options(options,img_path1,img_path2,homography_path,argc,argv))
    return -1;

  // Read image 1 and if necessary convert to grayscale.
  img1 = cv::imread(img_path1, 0);
  if (img1.data == NULL) {
    std::cerr << "Error loading image 1: " << img_path1 << std::endl;
    return -1;
  }

  // Read image 2 and if necessary convert to grayscale.
  img2 = cv::imread(img_path2, 0);
  if (img2.data == NULL) {
    std::cerr << "Error loading image 2: " << img_path2 << std::endl;
    return -1;
  }

  // Read ground truth homography file
  bool use_ransac = false;
  if (akaze_features::read_homography(homography_path, HG) == false)
    use_ransac = true;

  // Convert the images to float
  img1.convertTo(img1_32, CV_32F, 1.0/255.0, 0);
  img2.convertTo(img2_32, CV_32F, 1.0/255.0, 0);

  // Color images for results visualization
  img1_rgb = cv::Mat(cv::Size(img1.cols, img1.rows), CV_8UC3);
  img2_rgb = cv::Mat(cv::Size(img2.cols, img1.rows), CV_8UC3);
  img_com = cv::Mat(cv::Size(img1.cols*2, img1.rows), CV_8UC3);
  img_r = cv::Mat(cv::Size(img_com.cols*rfactor, img_com.rows*rfactor), CV_8UC3);

  // Create the first AKAZE object
  options.img_width = img1.cols;
  options.img_height = img1.rows;
  akaze_features::AKAZE evolution1(options);

  // Create the second HKAZE object
  options.img_width = img2.cols;
  options.img_height = img2.rows;
  akaze_features::AKAZE evolution2(options);

  t1 = cv::getTickCount();

  evolution1.Create_Nonlinear_Scale_Space(img1_32);
  evolution1.Feature_Detection(kpts1);
  evolution1.Compute_Descriptors(kpts1, desc1);

  evolution2.Create_Nonlinear_Scale_Space(img2_32);
  evolution2.Feature_Detection(kpts2);
  evolution2.Compute_Descriptors(kpts2, desc2);

  t2 = cv::getTickCount();
  takaze = 1000.0*(t2-t1)/cv::getTickFrequency();

  // Matching Descriptors!!
  std::vector<cv::Point2f> matches, inliers;
  cv::Ptr<cv::DescriptorMatcher> matcher_l2 = cv::DescriptorMatcher::create("BruteForce");
  cv::Ptr<cv::DescriptorMatcher> matcher_l1 = cv::DescriptorMatcher::create("BruteForce-Hamming");

  t1 = cv::getTickCount();

  if (options.descriptor < akaze_features::MLDB_UPRIGHT)
    matcher_l2->knnMatch(desc1, desc2, dmatches, 2);
  else
    matcher_l1->knnMatch(desc1, desc2, dmatches, 2);

  t2 = cv::getTickCount();
  tmatch = 1000.0*(t2 - t1)/ cv::getTickFrequency();

  // Compute Inliers!!
  akaze_features::matches2points_nndr(kpts1, kpts2, dmatches, matches, DRATIO);

  if (use_ransac == false)
    akaze_features::compute_inliers_homography(matches, inliers, HG, MIN_H_ERROR);
  else
    akaze_features::compute_inliers_ransac(matches, inliers, MIN_H_ERROR, false);

  // Compute the inliers statistics
  nkpts1 = kpts1.size();
  nkpts2 = kpts2.size();
  nmatches = matches.size()/2;
  ninliers = inliers.size()/2;
  noutliers = nmatches - ninliers;
  ratio = 100.0*((float) ninliers / (float) nmatches);

  // Prepare the visualization
  cv::cvtColor(img1, img1_rgb, cv::COLOR_GRAY2BGR);
  cv::cvtColor(img2, img2_rgb, cv::COLOR_GRAY2BGR);

  // Show matching statistics
  std::cout << "Number of Keypoints Image 1: " << nkpts1 << std::endl;
  std::cout << "Number of Keypoints Image 2: " << nkpts2 << std::endl;
  std::cout << "A-KAZE Features Extraction Time (ms): " << takaze << std::endl;
  std::cout << "Matching Descriptors Time (ms): " << tmatch << std::endl;
  std::cout << "Number of Matches: " << nmatches << std::endl;
  std::cout << "Number of Inliers: " << ninliers << std::endl;
  std::cout << "Number of Outliers: " << noutliers << std::endl;
  std::cout << "Inliers Ratio: " << ratio << std::endl << std::endl;

  akaze_features::draw_keypoints(img1_rgb, kpts1);
  akaze_features::draw_keypoints(img2_rgb, kpts2);
  akaze_features::draw_inliers(img1_rgb, img2_rgb, img_com, inliers);
  cv::namedWindow("Inliers", cv::WINDOW_NORMAL);
  cv::imshow("Inliers",img_com);
  cv::waitKey(0);
}

/* ************************************************************************* */
int parse_input_options(akaze_features::AKAZEOptions& options,
                        std::string& img_path1, std::string& img_path2,
                        std::string& homography_path,
                        int argc, char *argv[])
{

  // If there is only one argument return
  if (argc == 1) {
    akaze_features::show_input_options_help(1);
    return -1;
  }
  // Set the options from the command line
  else if (argc >= 2) {

    // Load the default options
    options = akaze_features::AKAZEOptions();

    if (!strcmp(argv[1],"--help")) {
      akaze_features::show_input_options_help(1);
      return -1;
    }

    img_path1 = argv[1];
    img_path2 = argv[2];

    if (argc >= 4)
     homography_path = argv[3];

    for (int i = 3; i < argc; i++) {
      if (!strcmp(argv[i],"--soffset")) {
        i = i+1;
        if (i >= argc) {
          std::cerr << "Error introducing input options!!" << std::endl;
          return -1;
        }
        else {
          options.soffset = atof(argv[i]);
        }
      }
      else if (!strcmp(argv[i],"--omax")) {
        i = i+1;
        if (i >= argc) {
          std::cerr << "Error introducing input options!!" << std::endl;
          return -1;
        }
        else {
          options.omax = atof(argv[i]);
        }
      }
      else if ( !strcmp(argv[i],"--dthreshold")) {
        i = i+1;
        if (i >= argc) {
          std::cerr << "Error introducing input options!!" << std::endl;
          return -1;
        }
        else {
          options.dthreshold = atof(argv[i]);
        }
      }
      else if (!strcmp(argv[i],"--sderivatives")) {
        i = i+1;
        if (i >= argc) {
          std::cerr << "Error introducing input options!!" << std::endl;
          return -1;
        }
        else {
          options.sderivatives = atof(argv[i]);
        }
      }
      else if (!strcmp(argv[i],"--nsublevels")) {
        i = i+1;
        if (i >= argc) {
          std::cerr << "Error introducing input options!!" << std::endl;
          return -1;
        }
        else {
          options.nsublevels = atoi(argv[i]);
        }
      }
      else if (!strcmp(argv[i],"--diffusivity"))
      {
        i = i+1;
        if (i >= argc) {
          std::cerr << "Error introducing input options!!" << std::endl;
          return -1;
        }
        else {
          options.diffusivity = akaze_features::DIFFUSIVITY_TYPE(atoi(argv[i]));
        }
      }
      else if (!strcmp(argv[i],"--descriptor")) {
        i = i+1;
        if (i >= argc) {
          std::cerr << "Error introducing input options!!" << std::endl;
          return -1;
        }
        else {
          options.descriptor = akaze_features::DESCRIPTOR_TYPE(atoi(argv[i]));

          if (options.descriptor < 0 ||
              options.descriptor > akaze_features::MLDB) {
            options.descriptor = akaze_features::MLDB;
          }
        }
      }
      else if (!strcmp(argv[i],"--descriptor_channels")) {
        i = i+1;
        if (i >= argc) {
          std::cerr << "Error introducing input options!!" << std::endl;
          return -1;
        }
        else {
          options.descriptor_channels = atoi(argv[i]);

          if (options.descriptor_channels <= 0 || options.descriptor_channels > 3 ) {
            options.descriptor_channels = 3;
          }
        }
      }
      else if (!strcmp(argv[i],"--descriptor_size")) {
        i = i+1;
        if (i >= argc) {
          std::cerr << "Error introducing input options!!" << std::endl;
          return -1;
        }
        else {
          options.descriptor_size = atoi(argv[i]);

          if (options.descriptor_size < 0) {
            options.descriptor_size = 0;
          }
        }
      }
      else if (!strcmp(argv[i],"--verbose")) {
        options.verbosity = true;
      }
      else if (!strncmp(argv[i],"--",2))
        std::cerr << "Unknown command "<<argv[i] << std::endl;
    }
  }
  else {
    std::cerr << "Error introducing input options!!" << std::endl;
    akaze_features::show_input_options_help(1);
    return -1;
  }

  return 0;
}
