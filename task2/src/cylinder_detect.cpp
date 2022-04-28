#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "laser_geometry/laser_geometry.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/MarkerArray.h"

// Depth map available at /camera/depth/image_raw
// Use opencv HoughCircles to detect circles in the depth map
// https://docs.opencv.org/4.x/da/d53/tutorial_py_houghcircles.html
// sudo apt install libopencv-dev

using namespace cv_bridge;

ros::Publisher image_pub;
ros::Publisher marker_pub;

laser_geometry::LaserProjection projector;

std::string OPENCV_WINDOW = "Camera";

tf2_ros::Buffer tfBuffer;

visualization_msgs::MarkerArray marker_array;

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

        // ROS_INFO("Found circle with radius %d", radius);
    }

    return valid_circles;
}

void handleScan(const sensor_msgs::LaserScan& scan) {

    // create empty output image
    cv::Mat mono8_img = cv::Mat::zeros(cv::Size(scan.ranges.size(), (int)(scan.ranges.size() * 0.75)), CV_8UC1);

    const double scale = 120;
    const double offset_x = scan.ranges.size();
    const double offset_y = scan.ranges.size() / 3;

    // convert polar coordinates to cartesian
    std::vector<cv::Point2f> points;
    for (size_t i = 0; i < scan.ranges.size(); i++) {  // only use every second point
        cv::Point2f point;
        point.x = -scan.ranges[i] * cos(scan.angle_min + scan.angle_increment * i) * scale + offset_x;
        point.y = scan.ranges[i] * sin(scan.angle_min + scan.angle_increment * i) * scale + offset_y;
        points.insert(points.end(), point);
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
    int accTreshold = 10;

    // detect circles in the depth map
    std::vector<cv::Vec3f> circles;
    circles = circleDetect(mono8_img, rgb_img, cannyTreshold, accTreshold);

    // cv::imshow(OPENCV_WINDOW, rgb_img);
    // cv::waitKey(1);

    // publish image with detections
    cv_bridge::CvImage out_msg;
    out_msg.header   = scan.header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    out_msg.image    = rgb_img;
    image_pub.publish(out_msg.toImageMsg());

    // use ros tf to convert from laserscan coordinates to map coordinates
    geometry_msgs::TransformStamped transformer;
    try {
        transformer = tfBuffer.lookupTransform("map", scan.header.frame_id, ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
    }

    // rescale and recenter detections, then convert them to map coordinates
    std::vector<cv::Point3f> cylinder_points;
    for (size_t i = 0; i < circles.size(); i++) {
        cv::Point3f point;
        point.x = (circles[i][0] - offset_x) / scale;
        point.y = (circles[i][1] - offset_y) / scale;
        point.z = 1;
        point.x += transformer.transform.translation.x;
        point.y += transformer.transform.translation.y;
        cylinder_points.insert(cylinder_points.end(), point);
    }

    // create markers
    for (size_t i = 0; i < cylinder_points.size(); i++) {
        // don't insert markers if they are close to existing markers
        bool insert = true;
        for (size_t j = 0; j < marker_array.markers.size(); j++) {
            double distance = sqrt(pow(marker_array.markers[j].pose.position.x - cylinder_points[i].x, 2) +
                                   pow(marker_array.markers[j].pose.position.y - cylinder_points[i].y, 2));
            if (distance < 0.5) {
                ROS_WARN("Marker too close to existing marker: %f", distance);
                insert = false;
                break;
            }
        }

        if (!insert) {
            continue;
        }

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = scan.header.stamp;
        marker.ns = "cylinders";
        marker.id = i + marker_array.markers.size();
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = cylinder_points[i].x;
        marker.pose.position.y = cylinder_points[i].y;
        marker.pose.position.z = cylinder_points[i].z;
        marker.lifetime = ros::Duration().fromSec(300);
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        ROS_INFO("Publishing new markers, now have %d", (int)marker_array.markers.size());
        marker_array.markers.insert(marker_array.markers.end(), marker);
        for (size_t j = 0; j < marker_array.markers.size(); j++) {
            ROS_INFO("Marker %d: %f, %f", (int)j, marker_array.markers[j].pose.position.x,
                        marker_array.markers[j].pose.position.y);
        }
    }

    marker_pub.publish(marker_array);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cylinder_detect");

    ROS_INFO("Starting cylinder detection");

    ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe ("/scan", 1, &handleScan);
    // ros::spin();

    image_pub = nh.advertise<sensor_msgs::Image>("/cylinder_detect/image", 1);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/cylinder_detect/markers", 10);

    sensor_msgs::LaserScanConstPtr scan_msg;

    tf2_ros::TransformListener tfListener(tfBuffer);

    // fetch new message periodically
    ros::Rate rate = ros::Rate(2);
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
