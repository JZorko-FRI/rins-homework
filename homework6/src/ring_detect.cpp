// Depth map available at /camera/depth/image_raw
// Use opencv HoughCircles to detect circles in the depth map
// https://docs.opencv.org/4.x/da/d53/tutorial_py_houghcircles.html
// sudo apt install libopencv-dev

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"

ros::Publisher pub;
std::string OPENCV_WINDOW = "Camera";

void handleImage(const sensor_msgs::ImageConstPtr& msg) {
    // ROS_INFO("Image received");
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg);
      ROS_INFO("copied image");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(100);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ring_detect");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe ("/camera/depth/image_raw", 1, &handleImage);
    // pub = nh.advertise<pcl::PCLPointCloud2>("rings_on_image", 1);
    ros::spin();
}
