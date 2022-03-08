#!/usr/bin/env python

## Simple publisher that outputs a string and an integer

import rospy
from homework1.msg import NamedValue

def talker():
    # Initialize publisher object with topic chatter and message type String
    pub = rospy.Publisher('my_demo', NamedValue, queue_size=10)
    # Initialize node with name 'talker'
    rospy.init_node('my_publisher', anonymous=True)
    # Set publishing rate in Hz
    rate = rospy.Rate(2)

    msg = NamedValue()
    msg.name = "Sent from my_publisher"
    msg.value = 0

    # Loop until node is killed
    while not rospy.is_shutdown():
        msg.value += 1
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
