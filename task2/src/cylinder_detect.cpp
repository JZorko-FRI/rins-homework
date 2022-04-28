#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "laser_geometry/laser_geometry.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// Depth map available at /camera/depth/image_raw
// Use opencv HoughCircles to detect circles in the depth map
// https://docs.opencv.org/4.x/da/d53/tutorial_py_houghcircles.html
// sudo apt install libopencv-dev

using namespace cv_bridge;

ros::Publisher image_pub;

laser_geometry::LaserProjection projector;

std::string OPENCV_WINDOW = "Camera";

std::vector<cv::Vec3f> circleDetect(cv::Mat input, cv::Mat output, int cannyTreshold, int accTreshold) {
    // prepare output arrays
    std::vector<cv::Vec3f> all_circles;
    std::vector<cv::Vec3f> valid_circles;

    // fixed arguments for HoughCircles
    int minRadius = 13;
    int maxRadius = 16;
    int accResolution = 1;
    int minDist = input.rows / 8;

    // perform detection
    cv::HoughCircles(input, all_circles, cv::HOUGH_GRADIENT, accResolution, minDist,
                                     cannyTreshold, accTreshold, minRadius, maxRadius);

    // if (all_circles.size() > 0) {
    //     ROS_INFO("Found circles");
    // }

    // draw detected circles
    for (size_t i = 0; i < all_circles.size(); i++) {
        cv::Point center(cvRound(all_circles[i][0]), cvRound(all_circles[i][1]));

        // store valid circles for output
        valid_circles.insert(valid_circles.end(), all_circles[i]);

        int radius = cvRound(all_circles[i][2]);
        // green circle center
        cv::circle(output, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        // red circle outline
        cv::circle(output, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);

        ROS_INFO("Found circle with radius %d", radius);
    }

    return valid_circles;
}

void handleScan(const sensor_msgs::LaserScan& scan) {

    // create empty output image
    cv::Mat mono8_img = cv::Mat::zeros(cv::Size(scan.ranges.size(), (int)(scan.ranges.size() * 0.75)), CV_8UC1);

    const double scale = 120;
    const double offset_x = 10;
    const double offset_y = scan.ranges.size() / 3;

    // convert polar coordinates to cartesian
    std::vector<cv::Point2f> points;
    for (size_t i = 0; i < scan.ranges.size(); i++) {  // only use every second point
        cv::Point2f point;
        point.x = scan.ranges[i] * cos(scan.angle_min + scan.angle_increment * i) * scale + offset_x;
        point.y = scan.ranges[i] * sin(scan.angle_min + scan.angle_increment * i) * scale + offset_y;
        points.insert(points.end(), point);
        // ROS_INFO("Point: %f, %f", point.x, point.y);
    }

    // display points on image plane
    for (size_t i = 0; i < points.size(); i+=2) {
        cv::circle(mono8_img, points[i], 1, cv::Scalar(255), -1, 8, 0);
    }

    // TODO: detect circles on image
    // create separate rgb image for displaying results
    cv::Mat rgb_img = cv::Mat(mono8_img.size(), CV_8UC3);
    cv::cvtColor(mono8_img, rgb_img, cv::COLOR_GRAY2RGB);

    // prepare arguments for circle detection
    int cannyTreshold = 50;
    int accTreshold = 9;

    // detect circles in the depth map
    std::vector<cv::Vec3f> scaled_circles;
    scaled_circles = circleDetect(mono8_img, rgb_img, cannyTreshold, accTreshold);

    cv::imshow(OPENCV_WINDOW, rgb_img);
    cv::waitKey(1);

    // // rescale and recenter detections
    // std::vector<cv::Vec3f> circles;
    // for (size_t i = 0; i < scaled_circles.size(); i++) {
    //     cv::Vec3f circle;
    //     circle[0] = scaled_circles[i][0] / scale - offset_x;
    //     circle[1] = scaled_circles[i][1] / scale - offset_y;
    //     circle[2] = scaled_circles[i][2] / scale;
    //     circles.insert(circles.end(), circle);
    // }

    // // TODO: convert detection to map coordinates
    // // use ros tf to convert from laserscan coordinates to map coordinates
    // for (size_t i = 0; i < circles.size(); i++) {
    //     tfBuffer.lookupTransform()

    //     .waitForTransform(
    //         scan->header.frame_id,
    //         "/base_link",
    //         scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
    //         ros::Duration(1.0))
    // }

    // TODO: publish markers


    // // TODO create marker for each circle

    // // display image with detections in separate window
    // cv::imshow(OPENCV_WINDOW, rgb_img);
    // cv::waitKey(1);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cylinder_detect");

    ROS_INFO("Starting cylinder detection");

    ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe ("/scan", 1, &handleScan);
    // ros::spin();

    image_pub = nh.advertise<sensor_msgs::Image>("/cylinder_detect/image", 1);

    sensor_msgs::LaserScanConstPtr scan_msg;

    // fetch new message periodically
    ros::Rate rate = ros::Rate(5);
    while (ros::ok())
    {
        scan_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan");
        if (scan_msg != NULL)
        {
            handleScan(*scan_msg);
        }
        rate.sleep();
    }

}
