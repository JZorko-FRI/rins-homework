#include <ros/ros.h> // Standard ROS class
#include "homework1/Custom.h"

// This is a callback function. It is executed every time a new message arrives
void messageReceived(const homework1::Custom::ConstPtr &message)
{
  ROS_INFO("Received: %s", message->content.c_str());
  return;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "homework1_subscriber"); // Initialize ROS system
  ros::NodeHandle node_handle;                  // Register this program as a ROS node

  // Create a subscriber
  ros::Subscriber subscriber = node_handle.subscribe("homework1/chat", 1000, &messageReceived);

  ros::spin(); // Repeat

  return 0;
}