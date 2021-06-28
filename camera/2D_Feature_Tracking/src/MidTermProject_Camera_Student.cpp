/* INCLUDES FOR THIS PROJECT */
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <sstream>
#include <vector>

#include "dataStructures.h"
#include "matching2D.hpp"

/* MAIN PROGRAM */
int main(int argc, const char *argv[]) {
  /* INIT VARIABLES AND DATA STRUCTURES */

  // data location
  std::string data_path = "../";

  // camera
  std::string img_base_path = data_path + "images/";
  std::string img_prefix =
      "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
  std::string img_file_type = ".png";
  int img_start_index = 0; // first file index to load (assumes Lidar and camera
                           // names have identical naming convention)
  int img_end_index = 9;   // last file index to load
  int img_fill_width =
      4; // no. of digits which make up the file index (e.g. img-0001.png)

  // misc
  int data_buffer_size = 2; // no. of images which are held in memory (ring
                            // buffer) at the same time
  std::vector<DataFrame> data_buffer; // list of data frames which are held in
                                      // memory at the same time
  bool b_vis = false;                 // visualize results

  /* MAIN LOOP OVER ALL IMAGES */

  for (size_t img_index = 0; img_index <= img_end_index - img_start_index;
       img_index++) {
    /* LOAD IMAGE INTO BUFFER */

    // assemble filenames for current index
    std::ostringstream img_number;
    img_number << std::setfill('0') << std::setw(img_fill_width)
               << img_start_index + img_index;
    std::string img_full_filename =
        img_base_path + img_prefix + img_number.str() + img_file_type;

    // load image from file and convert to grayscale
    cv::Mat img, img_gray;
    img = cv::imread(img_full_filename);
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

    //// STUDENT ASSIGNMENT
    //// TASK MP.1 -> replace the following code with ring buffer
    ///               of size dataBufferSize

    // push image into data frame buffer
    DataFrame frame;
    frame.cameraImg = img_gray;

    if (data_buffer.size() > data_buffer_size) {
      data_buffer.erase(data_buffer.begin());
    }
    data_buffer.push_back(frame);

    //// EOF STUDENT ASSIGNMENT
    std::cout << "#1 : LOAD IMAGE INTO BUFFER done" << std::endl;

    /* DETECT IMAGE KEYPOINTS */

    // extract 2D keypoints from current image
    // create empty feature list for current image
    std::vector<cv::KeyPoint> keypoints;
    std::string detector_type = "SHITOMASI";
    detector_type = "ORB";

    //// STUDENT ASSIGNMENT
    //// TODO(a-ngo): TASK MP.2 -> add the following keypoint detectors in file
    /// matching2D.cpp and enable string-based selection based on detectorType /
    ///-> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

    if (detector_type.compare("SHITOMASI") == 0) {
      detKeypointsShiTomasi(keypoints, img_gray, b_vis);
    } else if (detector_type.compare("HARRIS") == 0) {
      detKeypointsHarris(keypoints, img_gray, b_vis);
    } else if (detector_type.compare("FAST") == 0 ||
               detector_type.compare("BRISK") == 0 ||
               detector_type.compare("ORB") == 0 ||
               detector_type.compare("AKAZE") == 0 ||
               detector_type.compare("SIFT") == 0) {
      detKeypointsModern(keypoints, img_gray, detector_type);
    } else {
      std::cerr << detector_type << " not supported!" << std::endl;
    }

    //// EOF STUDENT ASSIGNMENT

    //// STUDENT ASSIGNMENT
    //// TODO(a-ngo): TASK MP.3 -> only keep keypoints on the preceding vehicle

    // only keep keypoints on the preceding vehicle
    bool b_focus_on_vehicle = true;
    cv::Rect vehicleRect(535, 180, 180, 150);
    if (b_focus_on_vehicle) {
      // ...
    }

    //// EOF STUDENT ASSIGNMENT

    // optional : limit number of keypoints (helpful for debugging and learning)
    // TODO(a-ngo): set false after testing
    bool b_limit_kpts = false;
    if (b_limit_kpts) {
      int max_keypoints = 20;

      if (detector_type.compare("SHITOMASI") ==
          0) { // there is no response info, so keep the first 50 as they are
               // sorted in descending quality order
        keypoints.erase(keypoints.begin() + max_keypoints, keypoints.end());
      }
      cv::KeyPointsFilter::retainBest(keypoints, max_keypoints);
      std::cout << " NOTE: Keypoints have been limited!" << std::endl;
    }

    // push keypoints and descriptor for current frame to end of data buffer
    (data_buffer.end() - 1)->keypoints = keypoints;
    std::cout << "#2 : DETECT KEYPOINTS done" << std::endl;

    /* EXTRACT KEYPOINT DESCRIPTORS */

    //// STUDENT ASSIGNMENT
    //// TODO(a-ngo): TASK MP.4 -> add the following descriptors in file
    /// matching2D.cpp and
    /// enable string-based selection based on descriptorType / -> BRIEF, ORB,
    /// FREAK, AKAZE, SIFT

    cv::Mat descriptors;
    std::string descriptor_type = "BRISK"; // BRIEF, ORB, FREAK, AKAZE, SIFT
    descKeypoints((data_buffer.end() - 1)->keypoints,
                  (data_buffer.end() - 1)->cameraImg, descriptors,
                  descriptor_type);
    //// EOF STUDENT ASSIGNMENT

    // push descriptors for current frame to end of data buffer
    (data_buffer.end() - 1)->descriptors = descriptors;

    std::cout << "#3 : EXTRACT DESCRIPTORS done" << std::endl;

    if (data_buffer.size() >
        1) // wait until at least two images have been processed
    {

      /* MATCH KEYPOINT DESCRIPTORS */

      std::vector<cv::DMatch> matches;
      std::string matcher_type = "MAT_BF";        // MAT_BF, MAT_FLANN
      std::string descriptor_type = "DES_BINARY"; // DES_BINARY, DES_HOG
      std::string selector_type = "SEL_NN";       // SEL_NN, SEL_KNN

      //// STUDENT ASSIGNMENT
      //// TODO(a-ngo): TASK MP.5 -> add FLANN matching in file matching2D.cpp
      //// TODO(a-ngo): TASK MP.6 -> add KNN match selection and perform
      /// descriptor distance
      /// ratio filtering with t=0.8 in file matching2D.cpp

      matchDescriptors((data_buffer.end() - 2)->keypoints,
                       (data_buffer.end() - 1)->keypoints,
                       (data_buffer.end() - 2)->descriptors,
                       (data_buffer.end() - 1)->descriptors, matches,
                       descriptor_type, matcher_type, selector_type);

      //// EOF STUDENT ASSIGNMENT

      // store matches in current data frame
      (data_buffer.end() - 1)->kptMatches = matches;

      std::cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << std::endl;

      // visualize matches between current and previous image
      b_vis = true;
      if (b_vis) {
        cv::Mat match_img = ((data_buffer.end() - 1)->cameraImg).clone();
        cv::drawMatches((data_buffer.end() - 2)->cameraImg,
                        (data_buffer.end() - 2)->keypoints,
                        (data_buffer.end() - 1)->cameraImg,
                        (data_buffer.end() - 1)->keypoints, matches, match_img,
                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                        std::vector<char>(),
                        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        std::string window_name =
            "Matching keypoints between two camera images";
        cv::namedWindow(window_name, 7);
        cv::imshow(window_name, match_img);
        std::cout << "Press key to continue to next image" << std::endl;
        cv::waitKey(0); // wait for key to be pressed
      }
      b_vis = false;
    }

  } // eof loop over all images

  return 0;
}