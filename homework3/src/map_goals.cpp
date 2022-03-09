#include "ros/ros.h"

#include <nav_msgs/GetMap.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

float map_resolution = 0;
geometry_msgs::TransformStamped map_transform;

// http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Message location to the map
void mapCallback(const nav_msgs::OccupancyGridConstPtr &msg_map)
{
  int size_x = msg_map->info.width;
  int size_y = msg_map->info.height;

  if ((size_x < 3) || (size_y < 3))
  {
    ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
    return;
  }

  map_resolution = msg_map->info.resolution;
  map_transform.transform.translation.x = msg_map->info.origin.position.x;
  map_transform.transform.translation.y = msg_map->info.origin.position.y;
  map_transform.transform.translation.z = msg_map->info.origin.position.z;

  map_transform.transform.rotation = msg_map->info.origin.orientation;

  return;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "homework3_map_goals");
  ros::NodeHandle node_handle;

  ros::Subscriber subscriber = node_handle.subscribe("map", 10, &mapCallback);

  MoveBaseClient ac("move_base", true);

  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the \"move_base\" action server to come up.");
  }

  // 5 goal locations on the map
  float map_goals[5][2] = {{2.65, -1.10}, {0.55, -1.25}, {1.05, 1.40}, {-2.90, 0.90}, {0.95, -2.20}};

  // Move robot to every location
  for (int i = 0; i < (sizeof(map_goals) / sizeof(*map_goals)); i++)
  {
    move_base_msgs::MoveBaseGoal map_goal;
    map_goal.target_pose.header.frame_id = "map";
    map_goal.target_pose.pose.orientation.w = 1.0;
    map_goal.target_pose.pose.position.x = map_goals[i][0];
    map_goal.target_pose.pose.position.y = map_goals[i][1];
    map_goal.target_pose.header.stamp = ros::Time::now();

    ROS_INFO("Moving to (x: %f, y: %f).", map_goals[i][0], map_goals[i][1]);

    ac.sendGoal(map_goal);
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Robot moved to (x: %f, y: %f).", map_goals[i][0], map_goals[i][1]);
    else
      ROS_ERROR("The robot failed to move!");
      // return 1; // Continue ...
  }

  return 0;
}