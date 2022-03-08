#!/usr/bin/env python

## Simple subscriber that listens for a string and an integer

import rospy
from homework1.msg import NamedValue

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard "%s %d"', data.name, data.value)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('my_subscriber', anonymous=True)

    rospy.Subscriber('my_demo', NamedValue, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
