#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String # built in message type

def talker():
    # Initialize publisher object with topic name and message type
    pub = rospy.Publisher('chatter', String, queue_size=10)
    # Initialize node with name 'talker'
    rospy.init_node('talker', anonymous=True)
    # Set publishing rate in Hz
    rate = rospy.Rate(2)
    # Loop until node is killed
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
