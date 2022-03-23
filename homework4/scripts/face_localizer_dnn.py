#!/usr/bin/python3

import sys
import rospy
import dlib
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros

import message_filters
import actionlib

from os.path import dirname, join
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

#import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class face_localizer:
    def __init__(self):
        rospy.init_node('face_localizer', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # The function for performin HOG face detection
        #self.face_detector = dlib.get_frontal_face_detector()
        protoPath = join(
            dirname(__file__),
            "deploy.prototxt.txt")  # Structure of the neural network
        modelPath = join(dirname(__file__),
                         "res10_300x300_ssd_iter_140000.caffemodel"
                         )  # Parameters of the network

        self.face_net = cv2.dnn.readNetFromCaffe(
            protoPath, modelPath
        )  # Alternatively: Torch, PyTorch over tensor flow for training

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Marker array object used for showing markers in Rviz
        self.marker_array = MarkerArray()
        self.marker_num = 1

        # Subscribe to the image and/or depth topic
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        # self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        # Publisher for the visualization markers
        self.markers_pub = rospy.Publisher('face_markers',
                                           MarkerArray,
                                           queue_size=100)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        # Initialize sound handler
        self.soundhandle = SoundClient()

        # Initialize action client
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.ac.wait_for_server()

        # Variable that tells us which phase we are currently in - searching for face or confirming
        self.confirming = False
        self.confirming_rep = 0
        self.pose_array = []

        rospy.sleep(1)

    def get_pose(self, coords, dist, stamp):
        # Calculate the position of the detected face

        k_f = 554  # kinect focal length in pixels

        x1, x2, y1, y2 = coords

        face_x = self.dims[1] / 2 - (
            x1 + x2) / 2.  # offset from the center of the image
        face_y = self.dims[0] / 2 - (y1 + y2) / 2.

        # in pixels
        angle_to_target = np.arctan2(face_x, k_f)

        # Get the angles in the base_link relative coordinate system
        x, y = dist * np.cos(angle_to_target), dist * np.sin(angle_to_target)

        ### Define a stamped message for transformation - directly in "base_link"
        #point_s = PointStamped()
        #point_s.point.x = x
        #point_s.point.y = y
        #point_s.point.z = 0.3
        #point_s.header.frame_id = "base_link"
        #point_s.header.stamp = rospy.Time(0)

        # Define a stamped message for transformation - in the "camera rgb frame"
        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = stamp

        # Get the point in the "map" coordinate system
        try:
            point_world = self.tf_buf.transform(
                point_s, "map")  # Transform this point in this system "map"

            # Create a Pose object with the same position
            pose = Pose()
            pose.position.x = point_world.point.x
            pose.position.y = point_world.point.y
            pose.position.z = point_world.point.z
        except Exception as e:
            print(e)
            pose = None

        return pose

    def find_faces(self):
        # print('I got a new image!')

        if self.confirming:
            self.confirming_rep += 1

        # Get the next rgb and depth images that are posted from the camera
        # We don't need a subscriber, we can just wait for the message.
        try:
            rgb_image_message = rospy.wait_for_message("/camera/rgb/image_raw",
                                                       Image)
        except Exception as e:
            print(e)
            return 0

        try:
            depth_image_message = rospy.wait_for_message(
                "/camera/depth/image_raw", Image)
        except Exception as e:
            print(e)
            return 0

        # Convert the images into a OpenCV (numpy) format (also numpy)

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(
                rgb_image_message,
                "bgr8")  # bgr8 - how many channels, how many bits per channel
        except CvBridgeError as e:
            print(e)

        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_image_message, "32FC1")  # 32-bit floating point
        except CvBridgeError as e:
            print(e)

        # Set the dimensions of the image
        self.dims = rgb_image.shape
        h = self.dims[0]
        w = self.dims[1]

        # Tranform image to gayscale
        #gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

        # Do histogram equlization - not needed for colour images
        #img = cv2.equalizeHist(gray)

        # Detect the faces in the image
        #face_rectangles = self.face_detector(rgb_image, 0)
        # NN needs this kind of input (blob)
        # Resize to 300x300 pixels for NN, others are blob parameters
        blob = cv2.dnn.blobFromImage(cv2.resize(rgb_image, (300, 300)), 1.0,
                                     (300, 300), (104.0, 177.0, 123.0))
        self.face_net.setInput(blob)
        face_detections = self.face_net.forward(
        )  # Forward pass in the network

        for i in range(0, face_detections.shape[2]):
            confidence = face_detections[
                0, 0, i,
                2]  # the network's confidence that this is a correct detection
            if confidence > 0.5:  # we believe that this is a face
                box = face_detections[0, 0, i, 3:7] * np.array(
                    [w, h, w, h])  # scale back the original size
                box = box.astype('int')
                x1, y1, x2, y2 = box[0], box[1], box[2], box[3]

                # Extract region containing face
                face_region = rgb_image[y1:y2, x1:x2]

                # Visualize the extracted face (in new window)
                #cv2.imshow("ImWindow", face_region)
                #cv2.waitKey(1)

                # Find the distance to the detected face
                face_distance = float(np.nanmean(
                    depth_image[y1:y2, x1:x2]))  # ignore the nan values

                print('Distance to face', face_distance)

                # Get the time that the depth image was received
                depth_time = depth_image_message.header.stamp

                # Find the location of the detected face
                # Position of the face, distance to the face, the time at which the image was taken
                pose = self.get_pose((x1, x2, y1, y2), face_distance,
                                     depth_time)

                # Tells us if a face has already been recognized at this location
                recognized = False

                if pose is not None and not np.isnan(
                        pose.position.x) and not np.isnan(
                            pose.position.y) and not np.isnan(pose.position.z):
                    # Check if face already detected
                    b = np.array(
                        (pose.position.x, pose.position.y, pose.position.z))
                    for marker in self.marker_array.markers:
                        a = np.array(
                            (marker.pose.position.x, marker.pose.position.y,
                             marker.pose.position.z))
                        # We have recognized this face previously
                        recognized |= (np.linalg.norm(a - b) < 1)

                    if not recognized:
                        self.confirming = True
                        self.pose_array.append(pose)

                        # First detection of a new face
                        if self.confirming_rep > 0:
                            # Stop current goal execution (stop moving)
                            self.ac.cancel_all_goals()

                        # End of detection
                        if self.confirming_rep >= 5:
                            # Confirm that this is in fact a face
                            success_num = len(self.pose_array)
                            if success_num >= 4:
                                # Greet face
                                self.soundhandle.say('Hello!')

                                # Calculate mean position of face
                                mean_pose = Pose()
                                for pos in self.pose_array:
                                    mean_pose.position.x += pos.position.x
                                    mean_pose.position.y += pos.position.y
                                    mean_pose.position.z += pos.position.z

                                mean_pose.position.x /= success_num
                                mean_pose.position.y /= success_num
                                mean_pose.position.z /= success_num

                                # Create a marker used for visualization
                                self.marker_num += 1
                                marker = Marker()
                                marker.header.stamp = rospy.Time(0)
                                marker.header.frame_id = 'map'
                                marker.pose = mean_pose
                                marker.type = Marker.CUBE
                                marker.action = Marker.ADD
                                marker.frame_locked = False
                                marker.lifetime = rospy.Duration.from_sec(100)
                                marker.id = self.marker_num
                                marker.scale = Vector3(0.1, 0.1, 0.1)
                                marker.color = ColorRGBA(0, 1, 0, 1)
                                self.marker_array.markers.append(marker)

                                print(self.marker_array)

                                self.markers_pub.publish(self.marker_array)

                            # Reset variables
                            self.confirming = False
                            self.confirming_rep = 0
                            self.pose_array = []

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

    face_finder = face_localizer()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        face_finder.find_faces()
        rate.sleep()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
