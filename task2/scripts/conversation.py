#!/usr/bin/env python3

"""
Simple example showing how to use the SoundClient provided by libsoundplay,
in blocking, non-blocking, and explicit usage.
"""


import rospy
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest
import speech_recognition as sr

scanner_angles = [0, -2, 2.6, -0.5]
max_angle = 3.14

def move_arm(self, angles, duration=1):
    trajectory = JointTrajectory()
    trajectory.joint_names = ["arm_shoulder_pan_joint",
                              "arm_shoulder_lift_joint", "arm_elbow_flex_joint", "arm_wrist_flex_joint"]
    trajectory.points = [JointTrajectoryPoint(positions=angles, time_from_start=rospy.Duration(duration))]
    self.pub.publish(trajectory)


def conversation():
    # Deliver food

    # Initialize ROS SoundHandle
    soundhandle = SoundClient(blocking=True)
    # Initialize speech recognition
    r = sr.Recognizer()

    soundhandle.say('Here you go. Your order.')
    print('Here I am')

    soundhandle.say('Will you pay by cash or credit card?')

    # Wait for response - obtain audio from the microphone
    with sr.Microphone() as source:
        print("Say something")
        audio = r.listen(source)

    # Recognize speech using Google Speech Recognition
    try:
        payment_method = r.recognize_google(audio)
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))

    # Move robot arm accordingly
    if payment_method == 'card':
        soundhandle.say('Card')
        self.scanner_angles[0] = self.max_angle
    else:
        soundhandle.say('Cash')
        self.scanner_angles[0] = -self.max_angle

    move_arm(scanner_angles)

    soundhandle.say('Please rate our service from one to five.')

    # Wait for response - obtain audio from the microphone
    with sr.Microphone() as source:
        print("Say something")
        audio = r.listen(source)

    # Recognize speech using Google Speech Recognition
    try:
        rating = r.recognize_google(audio)
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))

    if rating in ('five', 'four'):
        soundhandle.say('I\'m glad you were satisfied.')
    else:
        soundhandle.say('We will do better next time.')

    soundhandle.say('Thank you. Good-bye')


if __name__ == '__main__':
    rospy.init_node('convo_node', anonymous=True)
    self.pub = rospy.Publisher('/turtlebot_arm/arm_controller/command', JointTrajectory, queue_size=1)
    conversation()
