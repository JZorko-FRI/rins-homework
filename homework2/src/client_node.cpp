#include "ros/ros.h"
#include "homework2/MoveTurtle.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "homework2_client");
  ros::NodeHandle node_handle;

  ros::ServiceClient client = node_handle.serviceClient<homework2::MoveTurtle>("homework2/move_turtle");

  std::string movement_type;
  int duration;
  ros::param::get("~movement_type", movement_type);
  ros::param::get("~duration", duration);

  homework2::MoveTurtle move_turtle;

  move_turtle.request.movement_type = movement_type;
  move_turtle.request.duration = duration;

  ros::service::waitForService("homework2/move_turtle", 1000);

  ROS_INFO("Sending: [\"%s\", %d]", move_turtle.request.movement_type.c_str(), move_turtle.request.duration);

  if (client.call(move_turtle))
  {
    ROS_INFO("Received: \"%s\"", move_turtle.response.movement_type.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service!");
    return 1;
  }

  return 0;
}