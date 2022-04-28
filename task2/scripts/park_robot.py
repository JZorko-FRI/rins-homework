#!/usr/bin/python3

from cv2 import threshold
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
import numpy as np


class Park_Robot():
    def __init__(self):

        rospy.init_node('park_robot', anonymous=True)

        self.map_normal = np.load(
            '/home/mokot/FRI/RazvojInteligentnihSistemov/ROS/src/task2/maps/normal_map.npy'
        )

        self.map_resolution = rospy.get_param('/map_server/map_resolution')
        self.map_origin = rospy.get_param('/map_server/map_origin')

        self.park_robot_subscrier = rospy.Subscriber("/park_robot",
                                                     MoveBaseGoal,
                                                     self.move_robot)

        self.arm_robot_publisher = rospy.Publisher("/arm_command",
                                                   String,
                                                   queue_size=1)

        self.park_robot_client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)
        self.park_robot_client.wait_for_server()
        print("Service to park the robot is active!")

    def move_robot(self, data):
        print(data)
        # find the closest point
        parking_x = int(np.round((data.target_pose.pose.position.x + self.map_origin) /
                        self.map_resolution))
        parking_y = int(np.round((data.target_pose.pose.position.y + self.map_origin) /
                        self.map_resolution))

        magnitude_x, magnitude_y, direction = self.find_wall(
            parking_x, parking_y)
        parking_fi = self.map_normal[0, magnitude_x, magnitude_y]

        if direction == '+x':
            magnitude_x -= 10
        elif direction == '-x':
            magnitude_x += 10
        elif direction == '+y':
            magnitude_y -= 10
        elif direction == '-y':
            magnitude_y += 10

        print(magnitude_x * self.map_resolution - self.map_origin, magnitude_y * self.map_resolution - self.map_origin)

        # TODO
        x = np.cos(parking_fi) * 5 + data.target_pose.pose.position.x
        y = np.sin(parking_fi) * 5 + data.target_pose.pose.position.y

        # move to that point
        map_goal = MoveBaseGoal()
        map_goal.target_pose.header.frame_id = "map"
        map_goal.target_pose.header.stamp = rospy.Time.now()
        map_goal.target_pose.pose.position.x = data.target_pose.pose.position.x
        map_goal.target_pose.pose.position.y = data.target_pose.pose.position.y
        map_goal.target_pose.pose.position.z = data.target_pose.pose.position.z
        map_goal.target_pose.pose.orientation.x = data.target_pose.pose.orientation.x
        map_goal.target_pose.pose.orientation.y = data.target_pose.pose.orientation.y
        map_goal.target_pose.pose.orientation.z = data.target_pose.pose.orientation.z
        map_goal.target_pose.pose.orientation.w = data.target_pose.pose.orientation.w

        self.park_robot_client.send_goal(map_goal)
        wait = self.park_robot_client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")

        print("Moved to (x: %.2f, y: %.2f)." %
              (map_goal.target_pose.pose.position.x,
               map_goal.target_pose.pose.position.y))

        # extend arm camera
        self.arm_robot_publisher.publish("extend")

        # park the robot into parking spot

        # retract arm camera
        # self.arm_robot_publisher.publish("extend")

        return

    def find_wall(self, x, y):
        threshold = 75
        for i in range(100):
            if self.map_normal[1, x + i, y] > threshold:
                return x + i, y, '+x'
            elif self.map_normal[1, x - i, y] > threshold:
                return x - i, y, '-x'
            elif self.map_normal[1, x, y + i] > threshold:
                return x, y + i, '+y'
            elif self.map_normal[1, x, y - i] > threshold:
                return x, y - i, '-y'
        return x, y


if __name__ == "__main__":
    park_robot = Park_Robot()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
