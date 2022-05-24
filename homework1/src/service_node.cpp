#include <ros/ros.h> // Standard ROS class
#include "homework1/Array.h"

#include <string>
#include <iostream>

// Calculate array sum and return
bool array_sum(homework1::Array::Request &request, homework1::Array::Response &response)
{
  int str_length = request.array.length();
  int arr[str_length] = {0};

  int j = 0, i, sum = 0;

  for (i = 0; request.array.c_str()[i] != '\0'; i++)
  {
    if (request.array.c_str()[i] == ',' || request.array.c_str()[i] == '[' || request.array.c_str()[i] == ']')
      continue;
    if (request.array.c_str()[i] == ' ')
    {
      j++;
    }
    else
    {
      arr[j] = arr[j] * 10 + (request.array.c_str()[i] - 48);
    }
  }

  for (i = 0; i <= j; i++)
  {
    sum += arr[i];
  }

  response.sum = sum;
  ROS_INFO("Array: %s\tSum: %d", request.array.c_str(), sum);
  return true;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "homework1_service"); // Initialize ROS system
  ros::NodeHandle node_handle;                // Register this program as a ROS node

  ros::ServiceServer service = node_handle.advertiseService("homework1/array_sum", array_sum);

  ROS_INFO("Service: Active"); // Send string as a log message
  ros::spin();                 // Repeat

  return 0;
}