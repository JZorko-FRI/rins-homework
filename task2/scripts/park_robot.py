#!/usr/bin/python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Park_Robot():
    def __init__(self):

        rospy.init_node('park_robot', anonymous=True)

        self.map_normal = np.load(
            '/home/parallels/FRI/ros/src/homework/task2/maps/normal_map.npy'
        )

        self.map_resolution = rospy.get_param('/map_server/map_resolution')
        self.map_origin = rospy.get_param('/map_server/map_origin')
        self.parking = self.create_parking()

        self.park_robot_subscrier = rospy.Subscriber("/park_robot", String,
                                                     self.move_robot)

        self.arm_robot_publisher = rospy.Publisher("/arm_command",
                                                   String,
                                                   queue_size=1)

        self.arm_robot_twist_publisher = rospy.Publisher('cmd_vel',
                                                         Twist,
                                                         queue_size=1000)

        self.cv_bridge = CvBridge()

        self.park_robot_client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)
        self.park_robot_client.wait_for_server()
        print("Service to park the robot is active!")

    def move_robot(self, data):
        color = data.data
        data = MoveBaseGoal()
        # find the closest point
        parking_x = int(
            np.round((data.target_pose.pose.position.x + self.map_origin) /
                     self.map_resolution))
        parking_y = int(
            np.round((data.target_pose.pose.position.y + self.map_origin) /
                     self.map_resolution))

        magnitude_x, magnitude_y = self.find_wall(parking_x, parking_y)
        parking_fi = self.map_normal[0, magnitude_x, magnitude_y]

        x = np.cos(parking_fi) * 5 + data.target_pose.pose.position.x
        y = np.sin(parking_fi) * 5 + data.target_pose.pose.position.y

        index = 2
        if color == 'blue':
            index = 0
        elif color == 'black':
            index = 1
        elif color == 'green':
            index = 2
        elif color == 'red':
            index = 3

        # move to that point
        map_goal = MoveBaseGoal()
        map_goal.target_pose.header.frame_id = "map"
        map_goal.target_pose.header.stamp = rospy.Time.now()
        map_goal.target_pose.pose.position.x = self.points[index][0]
        map_goal.target_pose.pose.position.y = self.points[index][1]
        map_goal.target_pose.pose.position.z = 0.0
        map_goal.target_pose.pose.orientation.x = 0.0
        map_goal.target_pose.pose.orientation.y = 0.0
        map_goal.target_pose.pose.orientation.z = self.points[index][2]
        map_goal.target_pose.pose.orientation.w = self.points[index][3]

        self.park_robot_client.cancel_all_goals()

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
        self.park(index)

        # retract arm camera
        self.arm_robot_publisher.publish("extend")

        rospy.signal_shutdown()
        return

    def park(self, index):
        try:
            image_message = rospy.wait_for_message("/arm_camera/rgb/image_raw",
                                                   Image)
        except Exception as e:
            print(e)
            return

        try:
            image = self.cv_bridge.imgmsg_to_cv2(image_message, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = image[:360, :]

        circles = cv2.HoughCircles(image,
                                   cv2.HOUGH_GRADIENT,
                                   1,
                                   20,
                                   param1=30,
                                   param2=10,
                                   minRadius=0,
                                   maxRadius=25)

        if circles is None:
            # cv2.imshow('detected circles', image)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
            return

        circle = circles[0][0]
        center = image.shape[1] // 2

        map_goal = MoveBaseGoal()

        center_margin = 20
        forward_margin = 30
        if circle[0] - center > center_margin:
            # print("Turning right")
            map_goal.target_pose.pose.orientation.z = -1
        elif circle[0] - center < -center_margin:
            # print("Turning left")
            map_goal.target_pose.pose.orientation.z = 1
        elif circle[1] < image.shape[0] - forward_margin:
            # print("Moving forward")
            map_goal.target_pose.pose.position.x = 0.2

        print("Moving forward")
        map_goal.target_pose.header.frame_id = "map"
        map_goal.target_pose.header.stamp = rospy.Time.now()
        map_goal.target_pose.pose.position.x = self.parking_points[index][0]
        map_goal.target_pose.pose.position.y = self.parking_points[index][1]
        map_goal.target_pose.pose.position.z = 0.0
        map_goal.target_pose.pose.orientation.x = 0.0
        map_goal.target_pose.pose.orientation.y = 0.0
        map_goal.target_pose.pose.orientation.z = self.parking_points[index][2]
        map_goal.target_pose.pose.orientation.w = self.parking_points[index][3]
        self.park_robot_client.cancel_all_goals()
        self.park_robot_client.send_goal(map_goal)
        self.park_robot_client.wait_for_result()
        print("Robot parked!")

        return

    def find_wall(self, x, y):
        threshold = 75
        for i in range(100):
            if self.map_normal[1, x + i, y] > threshold:
                return x + i, y
            elif self.map_normal[1, x - i, y] > threshold:
                return x - i, y
            elif self.map_normal[1, x, y + i] > threshold:
                return x, y + i
            elif self.map_normal[1, x, y - i] > threshold:
                return x, y - i
        return x, y

    def create_parking(self):
        self.parking_points = np.asanyarray([
            (-0.55, 1.55, 0.15, 1.0),
            (1.75, 0.95, -0.70, -0.70),
            (2.4, -0.42, 1.0, -0.10),
            (2.90, -0.90, 0.6, -0.80),
        ])  # target point in parking spot # (x, y, z, w)
        self.points = np.asanyarray([
            (-1.05, 1.55, 0.15, 1.0),
            (1.75, 0.45, -0.70, -0.70),
            (3.0, -0.42, 1.0, -0.10),
            (2.90, -0.40, 0.6, -0.80),
        ])  # point outside the parking # (x, y, z, w)
        return


if __name__ == "__main__":
    park_robot = Park_Robot()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
