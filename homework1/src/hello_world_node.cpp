#include <ros/ros.h> // Standard ROS class

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "homework1_hello_world"); // Initialize ROS system
  ros::NodeHandle node_handle;                       // Register this program as a ROS node

  ROS_INFO("Hello, ROS!"); // Send string as a log message
  return 0;
}