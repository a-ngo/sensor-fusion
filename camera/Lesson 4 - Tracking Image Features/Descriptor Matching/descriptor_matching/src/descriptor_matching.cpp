#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "structIO.hpp"

void matchDescriptors(cv::Mat &imgSource, cv::Mat &imgRef,
                      std::vector<cv::KeyPoint> &kPtsSource,
                      std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource,
                      cv::Mat &descRef, std::vector<cv::DMatch> &matches,
                      std::string descriptorType, std::string matcherType,
                      std::string selectorType) {
  // configure matcher
  bool crossCheck = true;
  cv::Ptr<cv::DescriptorMatcher> matcher;

  // TODO(a-ngo): or possible to use matches from arg?
  std::vector<std::vector<cv::DMatch>> knn_matches;

  if (matcherType.compare("MAT_BF") == 0) {
    int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING
                                                             : cv::NORM_L2;
    matcher = cv::BFMatcher::create(normType, crossCheck);
    std::cout << "BF matching cross-check=" << crossCheck;
  } else if (matcherType.compare("MAT_FLANN") == 0) {
    if (descSource.type() != CV_32F) { // OpenCV bug workaround : convert binary
                                       // descriptors to floating point due to a
                                       // bug in current OpenCV implementation
      descSource.convertTo(descSource, CV_32F);
      descRef.convertTo(descRef, CV_32F);
    }
    matcher = cv::FlannBasedMatcher::create();
    std::cout << "FLANN matching";
  }

  // perform matching task
  // nearest neighbor (best match)
  if (selectorType.compare("SEL_NN") == 0) {
    double t = static_cast<double>(cv::getTickCount());
    // Finds the best match for each descriptor in desc1
    matcher->match(descSource, descRef, matches);
    t = static_cast<double>(cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << " (NN) with n=" << matches.size() << " matches in "
              << 1000 * t / 1.0 << " ms" << std::endl;
  } else if (selectorType.compare("SEL_KNN") == 0) {
    // implement k-nearest-neighbor matching
    double t = static_cast<double>(cv::getTickCount());

    // k nearest neighbors (k=2) -> finds the two best matches for each
    // descriptor
    int k = 2;
    matcher->knnMatch(descSource, descRef, knn_matches, k);

    // TODO(a-ngo): filter matches using descriptor distance ratio test
    for (size_t i = 0; i < knn_matches.size(); ++i) {
      // calculate ratio between descriptor distances to best and second-best
      // match

      // apply tresholds - of 0.8 - to sort out matches -> to keep match or
      // remove it
    }

    // TODO(a-ngo): write result to matches?

    t = static_cast<double>(cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << " (NN) with n=" << knn_matches.size() << " matches in "
              << 1000 * t / 1.0 << " ms" << std::endl;
  }

  // visualize results
  cv::Mat matchImg = imgRef.clone();
  cv::drawMatches(imgSource, kPtsSource, imgRef, kPtsRef, matches, matchImg,
                  cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
                  cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

  std::string windowName =
      "Matching keypoints between two camera images (best 50)";
  cv::namedWindow(windowName, 7);
  cv::imshow(windowName, matchImg);
  cv::waitKey(0);
}

int main() {
  cv::Mat imgSource = cv::imread("../images/img1gray.png");
  cv::Mat imgRef = cv::imread("../images/img2gray.png");

  std::string data = "BRISK_large";
  data = "BRISK_small";
  // data = "SIFT"; // TODO(a-ngo): why not working?

  std::vector<cv::KeyPoint> kptsSource, kptsRef;
  readKeypoints(std::string("../dat/C35A5_KptsSource_" + data + ".dat").c_str(),
                kptsSource);
  readKeypoints(std::string("../dat/C35A5_KptsRef_" + data + ".dat").c_str(),
                kptsRef);

  cv::Mat descSource, descRef;
  readDescriptors(
      std::string("../dat/C35A5_DescSource_" + data + ".dat").c_str(),
      descSource);
  readDescriptors(std::string("../dat/C35A5_DescRef_" + data + ".dat").c_str(),
                  descRef);

  std::vector<cv::DMatch> matches;

  // MAT_BF or MAT_FLANN
  std::string matcherType = "MAT_BF";
  matcherType = "MAT_FLANN";

  // NORM_HAMMING or NORM_L2
  std::string descriptorType = "DES_BINARY";

  // SEL_NN or SEL_KNN
  std::string selectorType = "SEL_KNN";
  matchDescriptors(imgSource, imgRef, kptsSource, kptsRef, descSource, descRef,
                   matches, descriptorType, matcherType, selectorType);
}
