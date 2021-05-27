#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp> // for canny etc.
#include <opencv2/videoio.hpp>

using namespace std;

void createMatrix1() {
  // create matrix
  int nrows = 480, ncols = 640;
  cv::Mat m1_8u;
  m1_8u.create(nrows, ncols,
               CV_8UC1); // two-channel matrix with 8bit unsigned elements
  m1_8u.setTo(cv::Scalar(255)); // white

  // STUDENT TASK :
  // Create a variable of type cv::Mat* named m3_8u which has three channels
  // with a depth of 8bit per channel. Then, set the first channel to 255 and
  // display the result.

  // show result
  string windowName = "First steps in OpenCV (m1_8u)";
  cv::namedWindow(windowName, 1); // create window
  cv::imshow(windowName, m1_8u);
  cv::waitKey(0); // wait for keyboard input before continuing

  // STUDENT TASK :
  // Display the results from the STUDENT TASK above
}

void webcam() {
  cv::VideoCapture cap(0);
  cv::Mat img, img_canny;

  while (true) {
    cap.read(img);

    cv::Canny(img, img_canny, 25, 75);

    cv::imshow("Webcam", img_canny);
    cv::waitKey(1);
  }
}

int main() {
  // createMatrix1();
  webcam();
  return 0;
}