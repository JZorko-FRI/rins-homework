#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"

// Depth map available at /camera/depth/image_raw
// Use opencv HoughCircles to detect circles in the depth map
// https://docs.opencv.org/4.x/da/d53/tutorial_py_houghcircles.html
// sudo apt install libopencv-dev

using namespace cv_bridge;
using namespace message_filters;

ros::Publisher image_pub;
ros::Publisher color_pub;
std::string OPENCV_WINDOW = "Camera";

std::string detectColors(std::vector<cv::Vec3f> circles, cv::Mat output, cv::Mat rgb_img) {
	// Define center and radius of circles
	cv::Point center(cvRound(circles[0][0]), cvRound(circles[0][1]));
	int radius = cvRound(circles[0][2]);

	int x = int(cvRound(circles[0][0] - radius)) + 10;
	int y = int(cvRound(circles[0][1] - radius)) + 10;

	int img_w = rgb_img.size().width;
	int img_h = rgb_img.size().height;

	if (y < 0) {
		y = 0;
	}

	if (y > img_h) {
		y = img_h;
	}

	if (x < 0) {
		x = 0;
	}

	if (x > img_w) {
		x = img_w;
	}

	int w = (x + 2 * (radius + 10) >= img_w) ? (img_w - x) : (2 * (radius + 10));
	int h = (y + 2 * (radius + 10) >= 0) ? (img_h - y) : (2 * (radius + 10));
	cv::Rect rect(x, y, w, h);
	cv::Mat crop_ring = rgb_img(rect);

	// To HSV
	cv::Mat hsv_light;
	cv::cvtColor(crop_ring, hsv_light, CV_BGR2HSV);
	std::vector<cv::Mat> channels;
	cv::split(hsv_light, channels);
	channels[0] += 30;
	cv::merge(channels, hsv_light);

	cv::Mat hsv_threshold;
	cv::inRange(hsv_light, cv::Scalar(0, 5, 0), cv::Scalar(255, 100, 100), hsv_threshold);
	
	float mean_r = mean(crop_ring, hsv_threshold)[0];
	float mean_g = mean(crop_ring, hsv_threshold)[1];
	float mean_b = mean(crop_ring, hsv_threshold)[2];
	
	if (mean_r < 40 && mean_g < 40 && mean_b < 40) {
		cv::putText(output, "black", center, cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar::all(255));
		return "black";
	}

	float mean_hue_light = mean(hsv_light, hsv_threshold)[0];

	std::string color = "none";

	if (mean_hue_light > 170 || mean_hue_light < 50) {
		color = "red";
	} else if (mean_hue_light > 50 && mean_hue_light < 120) {
		color = "green";
	} else if (mean_hue_light > 120 && mean_hue_light < 170) {
		color = "blue";
	}

	cv::putText(output, color, center, cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar::all(255));
	return color;
}

std::vector<cv::Vec3f> circleDetect(cv::Mat input, cv::Mat output, int cannyTreshold, int accTreshold, int centerTreshold) {
  // prepare output arrays
  std::vector<cv::Vec3f> all_circles;
  std::vector<cv::Vec3f> valid_circles;

  // fixed arguments for HoughCircles
  int minRadius = 10;
  int maxRadius = 1000;
  int accResolution = 2;
  int minDist = 1;

  // perform detection
  cv::HoughCircles(input, all_circles, cv::HOUGH_GRADIENT, accResolution, minDist,
                   cannyTreshold, accTreshold, minRadius, maxRadius);

  // if (all_circles.size() > 0) {
  //   ROS_INFO("Found circles");
  // }

  // ROS_INFO("Number of circles %lu.", all_circles.size());

  // draw detected circles
  for (size_t i = 0; i < all_circles.size(); i++) {
    cv::Point center(cvRound(all_circles[i][0]), cvRound(all_circles[i][1]));

    // only interested in hollow circles
    int center_color = input.at<uchar>(center);
    if (center_color > centerTreshold) {
      ROS_INFO("Ignoring circle with center color: %d", center_color);
      continue;
    }

    // store valid circles for output
    valid_circles.insert(valid_circles.end(), all_circles[i]);

    int radius = cvRound(all_circles[i][2]);
    // green circle center
    cv::circle(output, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
    // red circle outline
    cv::circle(output, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);

	//cv::imshow(OPENCV_WINDOW, output);
  }

  return valid_circles;
}

void handleImage(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::ImageConstPtr& msgRGB) {
  // ROS_INFO("Image received");

  cv_bridge::CvImageConstPtr cv_ptr;
  cv_bridge::CvImageConstPtr cv_rgb_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    // ROS_INFO("copied image");
	cv_rgb_ptr = cv_bridge::toCvCopy(msgRGB, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
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

  // prepare arguments for circle detection
  int cannyTreshold = 100;
  int accTreshold = 75;
  int centerTreshold = 50;

  // detect circles in the depth map
  std::vector<cv::Vec3f> circles;
  circles = circleDetect(mono8_img, rgb_img, cannyTreshold, accTreshold, centerTreshold);

  std::string color = "none";
  if (circles.size() >= 1) {
	// ROS_INFO("Detecting colors");
	color = detectColors(circles, rgb_img, cv_rgb_ptr->image);
  }

  // publish image with detections
  cv_bridge::CvImage out_msg;
  out_msg.header   = msg->header; // Same timestamp and tf frame as input image
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
  out_msg.image    = rgb_img;
  image_pub.publish(out_msg.toImageMsg());

  // Publish ring color

  if (color == "green") {
    std_msgs::String str;
    str.data = color;
    color_pub.publish(str);
    ros::shutdown();
  }

  // publish detected circles
  // TODO publish all detected circles with own message type

  // display image with detections in separate window
  // cv::imshow(OPENCV_WINDOW, rgb_img);
  // cv::waitKey(1);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ring_detect");

  ROS_INFO("Starting ring detection");

  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
  TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(depth_sub, rgb_sub, 10);

  sync.registerCallback(boost::bind(&handleImage, _1, _2));

  image_pub = nh.advertise<sensor_msgs::Image>("/ring_detect/image", 1);
  color_pub = nh.advertise<std_msgs::String>("/park_robot", 1000);

  ros::spin();
}
