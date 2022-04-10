// Depth map available at /camera/depth/image_raw
// Use opencv HoughCircles to detect circles in the depth map
// https://docs.opencv.org/4.x/da/d53/tutorial_py_houghcircles.html
// sudo apt install libopencv-dev

// #include <ros/ros.h>
// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/imgproc.hpp"

// ros::Publisher pub;

// void callback(const pcl::PCLPointCloud2ConstPtr& cloud_blob) {

//     pub.publish(cloud_blob);

// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "ring_detect");
//     ros::NodeHandle nh;
//     ros::Subscriber sub = nh.subscribe ("/camera/depth/image_raw", 1, &callback);
//     pub = nh.advertise<pcl::PCLPointCloud2>("rings_on_image", 1);
//     ros::spin();
// }
