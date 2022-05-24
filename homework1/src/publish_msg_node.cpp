#include <ros/ros.h> // Standard ROS class
#include "homework1/Custom.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "homework1_publisher"); // Initialize ROS system
  ros::NodeHandle node_handle;                  // Register this program as a ROS node

  // Create a publisher
  ros::Publisher publisher = node_handle.advertise<homework1::Custom>("homework1/chat", 1000);

  ros::Rate rate(1); // Loop at 1Hz until the node is shutdown

  int count = 0;
  while (ros::ok())
  {
    homework1::Custom message; // Create the message
    std::stringstream ss;

    ss << "ROS - homework1 " << count;
    message.content = ss.str();

    publisher.publish(message); // Publish the message

    ROS_INFO("%s", message.content.c_str()); // Send string as a log message

    rate.sleep();

    count++;
  }

  return 0;
}