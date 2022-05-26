#!/usr/bin/python3

import sys
import rospy
import dlib
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros

import actionlib

from os.path import dirname, join

#import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from sound_play.libsoundplay import SoundClient

import torch
from torchvision import datasets, models, transforms

input_size = 224

data_transforms = {
    'val': transforms.Compose([
        transforms.Resize(input_size),
        transforms.CenterCrop(input_size),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ]),
}

class_dict = {0:'baklava',
              1:'pizza',
              2:'pomfri',
              3:'solata',
              4:'torta'}


def dist(pose1, pose2, min_dist):
    """
    Return true if poses closer that min_dist
    """
    p1 = np.array([pose1.x, pose1.y, pose1.z])
    p2 = np.array([pose2.x, pose2.y, pose2.z])
    dist = np.linalg.norm(p1 - p2)

    return dist != np.nan and dist < min_dist


def detected(pose, pose_array, min_dist=0.5):
    """
    Return true, if we have not yet detected the pose in pose_array
    """
    for temp in pose_array:
        if dist(pose.position, temp.position, min_dist):
            return False

    return True


def robot_location(msg):
    global robot_pose
    robot_pose = PoseWithCovarianceStamped()
    robot_pose = msg


class food_localizer:
    def __init__(self):
        rospy.init_node('food_localizer', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        model_path = join(dirname(__file__), "best_foodero_model.pt")

        self.model = torch.load(model_path)
        self.model.eval()

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Marker array object used for showing markers in Rviz
        self.marker_array = MarkerArray()
        self.marker_num = 1

        # Subscribe to the image and/or depth topic
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        # self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        # Publiser for the visualization markers
        self.markers_pub = rospy.Publisher('food_markers',
                                           MarkerArray,
                                           queue_size=1000)
        self.location_sub = rospy.Subscriber('/amcl_pose',
                                             PoseWithCovarianceStamped,
                                             robot_location)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        # Initialize action client
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.ac.wait_for_server()

        # Initialize sound handler
        self.soundhandle = SoundClient()

        self.pose_array = []

    def recognise_food(self, img_p):

        # Convert the image to a pytorch tensor
        img = data_transforms['val'](img_p).unsqueeze(0)
        pred = self.model(img)

        # Predict confidence per class
        pred_np = pred.cpu().detach().numpy().squeeze()

        # Get index of most confident class
        class_ind = np.argmax(pred_np)

        # Return food name and confidence
        return class_dict[class_ind], pred_np[class_ind]

    def get_pose(self, coords, dist, stamp):
        # Calculate the position of the detected food

        k_f = 554  # kinect focal length in pixels

        x1, x2, y1, y2 = coords

        food_x = self.dims[1] / 2 - (x1 + x2) / 2.
        food_y = self.dims[0] / 2 - (y1 + y2) / 2.

        angle_to_target = np.arctan2(food_x, k_f)

        # Get the angles in the base_link relative coordinate system
        x, y = dist * np.cos(angle_to_target), dist * np.sin(angle_to_target)

        # Define a stamped message for transformation - in the "camera rgb frame"
        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = stamp

        # Get the point in the "map" coordinate system
        try:
            point_world = self.tf_buf.transform(point_s, "map")

            # Create a Pose object with the same position
            pose = Pose()
            pose.position.x = point_world.point.x
            pose.position.y = point_world.point.y
            pose.position.z = point_world.point.z
        except Exception as e:
            print(e)
            pose = None

        return pose

    def find_foods(self):
        # print('I got a new image!')

        # Get the next rgb and depth images that are posted from the camera
        try:
            rgb_image_message = rospy.wait_for_message("/camera/rgb/image_raw", Image)
        except Exception as e:
            print(e)
            return 0

        try:
            depth_image_message = rospy.wait_for_message("/camera/depth/image_raw", Image)
        except Exception as e:
            print(e)
            return 0

        # Convert the images into a OpenCV (numpy) format

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_message, "32FC1")
        except CvBridgeError as e:
            print(e)

        # Set the dimensions of the image
        self.dims = rgb_image.shape
        h = self.dims[0]
        w = self.dims[1]

        # Tranform image to gayscale
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

        # TODO adjust tresholds
        thrs1 = 2000
        thrs2 = 4000
        edge = cv2.Canny(gray, thrs1, thrs2, apertureSize=5)

        # Fit contour to image and recognise food inside
        contours, hierarchy = cv2.findContours(edge, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Track maximum and return only best match
        # Set best_confidence to min treshold
        best_confidence = 1
        best_guess, best_contour = None, None

        # Detect food in each region separately
        for contour in contours:
            # Filter contours by circularity
            area = cv2.contourArea(contour)
            arclength = cv2.arcLength(contour, True)
            circularity = 4 * 3.14159 * area / (arclength * arclength)
            if circularity < 0.6:  # TODO adjust treshold
                continue
            # Find bounding rectangle, (x, y) is top left
            x, y, w, h = cv2.boundingRect(contour)
            # Extract region of interest
            roi = rgb_image[y:y + h, x:x + w]
            # Detect food in the region
            food_name, confidence = self.recognise_food(roi)
            if confidence > best_confidence:
                best_guess, best_confidence, best_contour = food_name, confidence, contour

        print(best_guess)

        # Break if no food detected
        if not best_contour:
            return

        # Calculate the position of the detected food
        x, y, w, h = cv2.boundingRect(best_contour)
        roi_rgb = rgb_image[y:y + h, x:x + w]
        # Visualize the extracted food
        cv2.imshow("ImWindow", roi_rgb)
        cv2.waitKey(10)
        # Calculate center of enclosing circle to find distance
        (x,y), radius = cv2.minEnclosingCircle(best_contour)
        food_distance = depth_image[int(y), int(x)]

        print('Distance to food', food_distance)

        # Get the time that the depth image was recieved
        depth_time = depth_image_message.header.stamp

        # Find the location of the detected food
        pose = self.get_pose((x, x+w, y, y+h), food_distance, depth_time)

        if pose is not None:
            # Check if pose is valid and if we have already detected it
            if detected(pose, self.pose_array):
                # Add pose to detected poses
                self.pose_array.append(pose)

                # Create a marker used for visualization
                self.marker_num += 1
                marker = Marker()
                marker.header.stamp = rospy.Time(0)
                marker.header.frame_id = 'map'
                marker.pose = pose
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD
                marker.frame_locked = False
                marker.lifetime = rospy.Duration.from_sec(300)
                marker.id = self.marker_num
                marker.scale = Vector3(0.2, 0.2, 0.2)
                marker.color = ColorRGBA(0, 1, 0, 1)
                marker.text = best_guess
                self.marker_array.markers.append(marker)

                self.markers_pub.publish(self.marker_array)

    def depth_callback(self, data):

        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

        # Do the necessairy conversion so we can visuzalize it in OpenCV

        image_1 = depth_image / np.nanmax(depth_image)
        image_1 = image_1 * 255

        image_viz = np.array(image_1, dtype=np.uint8)

        #cv2.imshow("Depth window", image_viz)
        #cv2.waitKey(1)

        #plt.imshow(depth_image)
        #plt.show()


def main():

    food_finder = food_localizer()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        food_finder.find_foods()
        rate.sleep()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
