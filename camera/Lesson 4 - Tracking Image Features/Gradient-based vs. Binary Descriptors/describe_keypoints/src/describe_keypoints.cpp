#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <vector>
// #include <opencv2/xfeatures2d/nonfree.hpp> // needed for older versions of
// opencv

void detect(cv::Mat img_source, cv::Mat img_gray, std::string detector_name,
            cv::Ptr<cv::FeatureDetector> detector,
            std::vector<cv::KeyPoint> keypoints,
            cv::Ptr<cv::DescriptorExtractor> descriptor, cv::Mat desc) {
  // detector
  double t = static_cast<double>(cv::getTickCount());

  detector->detect(img_gray, keypoints);
  t = static_cast<double>(cv::getTickCount() - t) / cv::getTickFrequency();
  std::cout << detector_name << " detector with n= " << keypoints.size()
            << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;

  // descriptor
  t = static_cast<double>(cv::getTickCount());

  descriptor->compute(img_gray, keypoints, desc);

  t = static_cast<double>(cv::getTickCount() - t) / cv::getTickFrequency();
  std::cout << detector_name << " descriptor in " << 1000 * t / 1.0 << " ms"
            << std::endl;

  // visualize results
  cv::Mat visImage = img_source.clone();
  cv::drawKeypoints(img_source, keypoints, visImage, cv::Scalar::all(-1),
                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  std::string windowName = detector_name + " Detection Results";
  cv::namedWindow(windowName, 1);
  imshow(windowName, visImage);
}

void descKeypoints1() {
  // load image from file and convert to grayscale
  cv::Mat imgGray;
  cv::Mat img = cv::imread("../images/img1.png");
  cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

  // BRISK detector / descriptor
  //   cv::Ptr<cv::FeatureDetector> detector = cv::BRISK::create();
  //   std::vector<cv::KeyPoint> kpts_BRISK;

  //   double t = static_cast<double>(cv::getTickCount());
  //   detector->detect(imgGray, kpts_BRISK);
  //   t = static_cast<double>(cv::getTickCount() - t) / cv::getTickFrequency();
  //   std::cout << "BRISK detector with n= " << kpts_BRISK.size()
  //             << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;

  //   cv::Ptr<cv::DescriptorExtractor> descriptor = cv::BRISK::create();
  //   cv::Mat descriptor_BRISK;
  //   t = static_cast<double>(cv::getTickCount());

  //   descriptor->compute(imgGray, kpts_BRISK, descriptor_BRISK);

  //   t = static_cast<double>(cv::getTickCount() - t) / cv::getTickFrequency();
  //   std::cout << "BRISK descriptor in " << 1000 * t / 1.0 << " ms" <<
  //   std::endl;

  //   // visualize results
  //   cv::Mat visImage = img.clone();
  //   cv::drawKeypoints(img, kpts_BRISK, visImage, cv::Scalar::all(-1),
  //                     cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  //   std::string windowName = "BRISK Results";
  //   cv::namedWindow(windowName, 1);
  //   imshow(windowName, visImage);

  // SIFT detector
  // detect keypoints
  //   cv::Ptr<cv::FeatureDetector> sift_detector = cv::SIFT::create();
  //   std::vector<cv::KeyPoint> sift_keypoints;

  //   t = static_cast<double>(cv::getTickCount());
  //   sift_detector->detect(imgGray, sift_keypoints);
  //   t = static_cast<double>(cv::getTickCount() - t) / cv::getTickFrequency();
  //   std::cout << "SIFT detector with n= " << sift_keypoints.size()
  //             << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;

  //   // descriptor
  //   cv::Ptr<cv::DescriptorExtractor> sift_descriptor = cv::SIFT::create();
  //   cv::Mat sift_descr;

  //   t = static_cast<double>(cv::getTickCount());

  //   sift_descriptor->compute(imgGray, sift_keypoints, sift_descr);

  //   t = static_cast<double>(cv::getTickCount() - t) / cv::getTickFrequency();
  //   std::cout << "SIFT descriptor in " << 1000 * t / 1.0 << " ms" <<
  //   std::endl;

  //   // visualize results
  //   visImage = img.clone();
  //   cv::drawKeypoints(img, sift_keypoints, visImage, cv::Scalar::all(-1),
  //                     cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  //   windowName = "SIFT Results";
  //   cv::namedWindow(windowName, 1);
  //   imshow(windowName, visImage);

  // BRISK
  cv::Ptr<cv::FeatureDetector> detector_brisk = cv::BRISK::create();
  std::vector<cv::KeyPoint> keypoints_brisk;
  cv::Ptr<cv::DescriptorExtractor> descriptor_brisk = cv::BRISK::create();
  cv::Mat descr_brisk;
  detect(img, imgGray, "BRISK", detector_brisk, keypoints_brisk,
         descriptor_brisk, descr_brisk);

  // SIFT
  cv::Ptr<cv::FeatureDetector> detector_sift = cv::SIFT::create();
  std::vector<cv::KeyPoint> keypoints_sift;
  cv::Ptr<cv::DescriptorExtractor> descriptor_sift = cv::SIFT::create();
  cv::Mat descr_sift;
  detect(img, imgGray, "SIFT", detector_sift, keypoints_sift, descriptor_sift,
         descr_sift);

  // ORB
  cv::Ptr<cv::FeatureDetector> detector_orb = cv::ORB::create();
  std::vector<cv::KeyPoint> keypoints_orb;
  cv::Ptr<cv::DescriptorExtractor> descriptor_orb = cv::ORB::create();
  cv::Mat descr_orb;
  detect(img, imgGray, "ORB", detector_orb, keypoints_orb, descriptor_orb,
         descr_orb);

  cv::waitKey(0);

  // Add the SIFT detector / descriptor, compute the
  // time for both steps and compare both BRISK and SIFT
  // with regard to processing speed and the number and
  // visual appearance of keypoints.
}

int main() {
  descKeypoints1();
  return 0;
}
