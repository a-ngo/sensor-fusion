#include "matching2D.hpp"
#include <numeric>

// Find best matches for keypoints in two camera images based on several
// matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &k_pts_source,
                      std::vector<cv::KeyPoint> &k_pts_ref,
                      cv::Mat &desc_source, cv::Mat &desc_ref,
                      std::vector<cv::DMatch> &matches,
                      std::string descriptor_type, std::string matcher_type,
                      std::string selector_type) {
  // configure matcher
  bool cross_check = false;
  cv::Ptr<cv::DescriptorMatcher> matcher;

  if (matcher_type.compare("MAT_BF") == 0) {
    int norm_type = cv::NORM_HAMMING;
    matcher = cv::BFMatcher::create(norm_type, cross_check);
  } else if (matcher_type.compare("MAT_FLANN") == 0) {
    // ...
  }

  // perform matching task
  if (selector_type.compare("SEL_NN") == 0) { // nearest neighbor (best match)
    matcher->match(
        desc_source, desc_ref,
        matches); // Finds the best match for each descriptor in desc1
  } else if (selector_type.compare("SEL_KNN") ==
             0) { // k nearest neighbors (k=2)

    // ...
  }
}

// Use one of several types of state-of-art descriptors to uniquely identify
// keypoints
void descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img,
                   cv::Mat &descriptors, std::string descriptor_type) {
  // select appropriate descriptor
  cv::Ptr<cv::DescriptorExtractor> extractor;
  if (descriptor_type.compare("BRISK") == 0) {

    int threshold = 30;        // FAST/AGAST detection threshold score.
    int octaves = 3;           // detection octaves (use 0 to do single scale)
    float patternScale = 1.0f; // apply this scale to the pattern used for
                               // sampling the neighbourhood of a keypoint.

    extractor = cv::BRISK::create(threshold, octaves, patternScale);
  } else {

    //...
  }

  // perform feature description
  double t = (double)cv::getTickCount();
  extractor->compute(img, keypoints, descriptors);
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  std::cout << descriptor_type << " descriptor extraction in " << 1000 * t / 1.0
            << " ms" << std::endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img,
                           bool b_vis) {
  // compute detector parameters based on image size
  int block_size = 4; //  size of an average block for computing a derivative
                      //  covariation matrix over each pixel neighborhood
  double max_overlap =
      0.0; // max. permissible overlap between two features in %
  double min_distance = (1.0 - max_overlap) * block_size;
  int max_corners = img.rows * img.cols /
                    std::max(1.0, min_distance); // max. num. of keypoints

  double quality_level = 0.01; // minimal accepted quality of image corners
  double k = 0.04;

  // Apply corner detection
  double t = static_cast<double>(cv::getTickCount());
  std::vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(img, corners, max_corners, quality_level,
                          min_distance, cv::Mat(), block_size, false, k);

  // add corners to result vector
  for (auto it = corners.begin(); it != corners.end(); ++it) {
    cv::KeyPoint new_key_point;
    new_key_point.pt = cv::Point2f((*it).x, (*it).y);
    new_key_point.size = block_size;
    keypoints.push_back(new_key_point);
  }
  t = static_cast<double>(cv::getTickCount() - t) / cv::getTickFrequency();
  std::cout << "Shi-Tomasi detection with n=" << keypoints.size()
            << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;

  // visualize results
  if (b_vis) {
    cv::Mat vis_image = img.clone();
    cv::drawKeypoints(img, keypoints, vis_image, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    std::string window_name = "Shi-Tomasi Corner Detector Results";
    cv::namedWindow(window_name, 6);
    imshow(window_name, vis_image);
    cv::waitKey(0);
  }
}
