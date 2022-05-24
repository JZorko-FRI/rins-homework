#include <ros/ros.h> // Standard ROS class
#include "homework1/Array.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "homework1_client"); // Initialize ROS system
  ros::NodeHandle node_handle;               // Register this program as a ROS node

  ros::ServiceClient client = node_handle.serviceClient<homework1::Array>("homework1/array_sum");

  //Seed the random number generator.
	srand(time(0));

  std::string arr = "";
  arr += '[';
  for (int i = 0; i < 10; i++)
  {
    arr += std::to_string(rand() % 100 + 1) + ", ";
  }
  arr += ']';

  std::cout << arr;

  homework1::Array srv;
  std::stringstream ss;

  ss << arr;
  srv.request.array = ss.str();

  ros::service::waitForService("homework1/array_sum", 1000);

  ROS_INFO("Sending: %s", srv.request.array.c_str());

  if (client.call(srv))
  {
    ROS_INFO("Received: %d", srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service!");
    return 1;
  }

  return 0;
}