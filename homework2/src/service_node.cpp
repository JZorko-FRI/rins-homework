#include "ros/ros.h"
#include "homework2/MoveTurtle.h"
#include <geometry_msgs/Twist.h>

float linear_x, linear_y, linear_z, angular_x, angular_y, angular_z;
int step;

const struct
{
  std::string CIRCLE;
  std::string RECTANGLE;
  std::string TRIANGLE;
  std::string RANDOM;
} const_movement_type = {"CIRCLE", "RECTANGLE", "TRIANGLE", "RANDOM"};

bool move_turtle(homework2::MoveTurtle::Request &request, homework2::MoveTurtle::Response &response)
{
  response.movement_type = request.movement_type;
  ROS_INFO("Movement Type: \"%s\"\tDuration: %d", request.movement_type.c_str(), request.duration);

  ros::NodeHandle node_handle;
  ros::Publisher publisher = node_handle.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

  ros::Time timer = ros::Time::now();

  ros::Rate rate(1);

  srand(time(0));

  std::string movement_type = request.movement_type.c_str();
  if (!(movement_type.compare(const_movement_type.CIRCLE)))
  {
    linear_x = 2, linear_y = 0.0, linear_z = 0.0;
    angular_x = 0.0, angular_y = 0.0, angular_z = 1.0;
  }
  else if (!(movement_type.compare(const_movement_type.RECTANGLE)))
  {
    linear_x = 1.0, linear_y = 0.0, linear_z = 0.0;
    angular_x = 0.0, angular_y = 0.0, angular_z = 0.0;
    step = 0;
  }
  else if (!(movement_type.compare(const_movement_type.TRIANGLE)))
  {
    linear_x = 1.0, linear_y = 0.0, linear_z = 0.0;
    angular_x = 0.0, angular_y = 0.0, angular_z = 0.0;
    step = 0;
  }
  else
  {
    movement_type = const_movement_type.RANDOM;
    linear_x = double(rand()) / double(RAND_MAX), linear_y = 0.0, linear_z = 0.0;
    angular_x = 0.0, angular_y = 0.0, angular_z = double(rand()) / double(RAND_MAX);
  }

  while ((ros::Time::now().toSec() - timer.toSec()) < (request.duration + 0.1))
  {
    if (!(movement_type.compare(const_movement_type.RECTANGLE)))
    {
      step %= 5;
      if (step % 5 == 0)
      {
        linear_x = 0.0;
        angular_z = 1.57; // 90 / 360 * 2 * 3.14;
      }
      else
      {
        angular_z = 0;
        linear_x = 1.0;
      }
    }

    if (!(movement_type.compare(const_movement_type.TRIANGLE)))
    {
      step %= 5;
      if (step % 5 == 0)
      {
        linear_x = 0.0;
        angular_z = 2.09; // 120 / 360 * 2 * 3.14;
      }
      else
      {
        angular_z = 0;
        linear_x = 1.0;
      }
    }

    if (!(movement_type.compare(const_movement_type.RANDOM)))
    {
      linear_x = double(rand()) / double(RAND_MAX), linear_y = 0.0, linear_z = 0.0;
      angular_x = 0.0, angular_y = 0.0, angular_z = double(rand()) / double(RAND_MAX);
    }

    geometry_msgs ::Twist msg;
    msg.linear.x = linear_x;
    msg.linear.y = linear_y;
    msg.linear.z = linear_z;
    msg.angular.x = angular_x;
    msg.angular.y = angular_y;
    msg.angular.z = angular_z;

    publisher.publish(msg);

    ROS_INFO(
        "Velocity command: %s \"linear: [x = %.1f, y = %.1f, z = %.1f], angular: [x = %.1f, y = %.1f, z = %.1f]\"",
        request.movement_type.c_str(),
        msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z);

    step += 1;
    rate.sleep();
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "homework2_service");
  ros::NodeHandle node_handle;

  linear_x = 0.0, linear_y = 0.0, linear_z = 0.0;
  angular_x = 0.0, angular_y = 0.0, angular_z = 0.0;
  step = 1;

  ros::ServiceServer service = node_handle.advertiseService("homework2/move_turtle", move_turtle);

  ROS_INFO("Service: Active");
  ros::spin();

  return 0;
}