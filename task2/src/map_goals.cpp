#include "ros/ros.h"

#include <nav_msgs/GetMap.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>

#include <sound_play/sound_play.h>
#include <unistd.h>

using namespace std;

#define STATUS_REPORT_INTERVAL 3.0
#define GLOAL_TIMEOUT 30.0

float map_resolution = 0;
bool moving = false;
geometry_msgs::TransformStamped map_transform;

// http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void sleepok(int t, ros::NodeHandle &nh)
{
  if (nh.ok())
    sleep(t);
}

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

void statusCallback(const actionlib_msgs::GoalStatusArray &goal_status)
{
  if (goal_status.header.seq % 10 == 0 && moving)
  {
    ROS_INFO("%s", (char *)&goal_status.status_list[0].text);
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "task2_map_goals");
  ros::NodeHandle node_handle;

  ros::Subscriber map_subscriber = node_handle.subscribe("map", 10, &mapCallback);
  ros::Subscriber goal_subscriber = node_handle.subscribe("/move_base/status", 10, &statusCallback);

  MoveBaseClient ac("move_base", true);
  sound_play::SoundClient sound_client;

  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the \"move_base\" action server to come up.");
  }

  // 5 goal locations on the map
  const float map_goals[9][2] = {
      {-0.85, -0.20},
      {-0.75, 1.55},
      {1.00, 2.40},
      {1.30, 0.25},
      {2.05, 1.00},
      {3.30, -0.15},
      {2.60, 0.20},
      {2.25, -1.30},
      {1.55, -1.55},
  };

  // Move robot to every location
  int i = 0;
  while (i < (sizeof(map_goals) / sizeof(*map_goals)))
  {
    move_base_msgs::MoveBaseGoal map_goal;
    map_goal.target_pose.header.frame_id = "map";
    map_goal.target_pose.pose.orientation.w = 1.0;
    map_goal.target_pose.pose.position.x = map_goals[i][0];
    map_goal.target_pose.pose.position.y = map_goals[i][1];
    map_goal.target_pose.header.stamp = ros::Time::now();

    ROS_INFO("Moving to (x: %f, y: %f).", map_goals[i][0], map_goals[i][1]);

    ac.sendGoal(map_goal);

    moving = true;
    ac.waitForResult();
    moving = false;

    actionlib::SimpleClientGoalState goal_state = ac.getState();

    if (goal_state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Robot moved to (x: %f, y: %f).", map_goals[i][0], map_goals[i][1]);
      i++;
    }
    else if (goal_state == actionlib::SimpleClientGoalState::RECALLED || goal_state == actionlib::SimpleClientGoalState::PREEMPTED)
    {
      ROS_INFO("Face has been detected. Waiting for confirmation.");
      ros::Duration(3).sleep();
    }
    else
    {
      ROS_ERROR("The robot failed to move!");
      i++;
      // return 1; // Continue ...
    }
  }

  std::string sounds_dir;
  node_handle.param<std::string>("/sound_play/sounds_dir", sounds_dir, "./");

  // At the end play star wars
  // sound_client.playWave(sounds_dir + "star-wars-theme-song.wav");
  sleepok(30, node_handle);

  return 0;
}
