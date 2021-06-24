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
  bool crossCheck = false;
  cv::Ptr<cv::DescriptorMatcher> matcher;

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

    //... TODO : implement FLANN matching
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
    // k nearest neighbors (k=2)

    // TODO(a-ngo): implement k-nearest-neighbor matching

    // TODO(a-ngo): filter matches using descriptor distance ratio test
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

  std::vector<cv::KeyPoint> kptsSource, kptsRef;
  readKeypoints("../dat/C35A5_KptsSource_BRISK_large.dat", kptsSource);
  readKeypoints("../dat/C35A5_KptsRef_BRISK_large.dat", kptsRef);

  cv::Mat descSource, descRef;
  readDescriptors("../dat/C35A5_DescSource_BRISK_large.dat", descSource);
  readDescriptors("../dat/C35A5_DescRef_BRISK_large.dat", descRef);

  std::vector<cv::DMatch> matches;
  std::string matcherType = "MAT_BF";
  std::string descriptorType = "DES_BINARY";
  std::string selectorType = "SEL_NN";
  matchDescriptors(imgSource, imgRef, kptsSource, kptsRef, descSource, descRef,
                   matches, descriptorType, matcherType, selectorType);
}
