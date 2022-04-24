#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"

// Depth map available at /camera/depth/image_raw
// Use opencv HoughCircles to detect circles in the depth map
// https://docs.opencv.org/4.x/da/d53/tutorial_py_houghcircles.html
// sudo apt install libopencv-dev

using namespace cv_bridge;

ros::Publisher pub;
std::string OPENCV_WINDOW = "Camera";

void circleDetect(cv::Mat input, cv::Mat output, int cannyTreshold, int accTreshold, int centerTreshold) {
  std::vector<cv::Vec3f> circles;
  int minRadius = 10;
  int maxRadius = 1000;
  int accResolution = 2;
  int minDist = input.rows / 8;
  cv::HoughCircles(input, circles, cv::HOUGH_GRADIENT, accResolution, minDist,
                   cannyTreshold, accTreshold, minRadius, maxRadius);

  if (circles.size() > 0) {
    ROS_INFO("Found circles");
  }

  for (size_t i = 0; i < circles.size(); i++) {
    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));

    // only interested in hollow circles
    int center_color = input.at<uchar>(center);
    if (center_color > centerTreshold) {
      ROS_INFO("Ignoring circle with center color: %d", center_color);
      continue;
    }

    int radius = cvRound(circles[i][2]);
    // circle center
    cv::circle(output, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
    // circle outline
    cv::circle(output, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
  }
}

void handleImage(const sensor_msgs::ImageConstPtr& msg) {
    // ROS_INFO("Image received");
    CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = toCvCopy(msg);
      // ROS_INFO("copied image");
    }
    catch (Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // message contaings 32-bit floating point depth image
    // convert to standard 8-bit gray scale image
    cv::Mat mono8_img = cv::Mat(cv_ptr->image.size(), CV_8UC1);
    cv::convertScaleAbs(cv_ptr->image, mono8_img, 100, 0.0);

    // create separate rgb image for displaying results
    cv::Mat rgb_img = cv::Mat(mono8_img.size(), CV_8UC3);
    cv::cvtColor(mono8_img, rgb_img, cv::COLOR_GRAY2RGB);

    // detect circles in the depth map
    int cannyTreshold = 100;
    int accTreshold = 75;
    int centerTreshold = 50;
    circleDetect(mono8_img, rgb_img, cannyTreshold, accTreshold, centerTreshold);

    cv::imshow(OPENCV_WINDOW, rgb_img);
    cv::waitKey(1);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ring_detect");

    ROS_INFO("Starting ring detection");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe ("/camera/depth/image_raw", 1, &handleImage);
    // pub = nh.advertise<pcl::PCLPointCloud2>("rings_on_image", 1);
    ros::spin();
}
