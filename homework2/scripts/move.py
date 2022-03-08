#!/usr/bin/env python

# Simple usage: rosservice call /move "{shape: square, duration: 10}"

import rospy
import random

from time import time

from geometry_msgs.msg import Twist
from homework2.srv import Command, CommandResponse

scale = rospy.get_param('scale', 2)

def _deg2rad(deg):
    return deg / 180 * 3.14159

def _move_circle(publisher, duration, rate):
    circle_twist = Twist()
    circle_twist.linear.x = 1
    circle_twist.angular.z = _deg2rad(90) / scale
    steps = 0
    while not rospy.is_shutdown() and steps < duration:
        publisher.publish(circle_twist)
        rate.sleep()
        steps += 1

straight_twist = Twist()
straight_twist.linear.x = 1

def _move_triangle(publisher, duration, rate):
    rotate_left_twist = Twist()
    rotate_left_twist.angular.z = _deg2rad(120)

    steps = 0
    while not rospy.is_shutdown() and steps < duration:
        if steps % (scale + 1) == 0:
            publisher.publish(rotate_left_twist)
        else:
            publisher.publish(straight_twist)
        rate.sleep()
        steps += 1

def _move_square(publisher, duration, rate):
    rotate_left_twist = Twist()
    rotate_left_twist.angular.z = _deg2rad(90)

    steps = 0
    while not rospy.is_shutdown() and steps < duration:
        if steps % (scale + 1) == 0:
            publisher.publish(rotate_left_twist)
        else:
            publisher.publish(straight_twist)
        rate.sleep()
        steps += 1

def _move_random(publisher, duration, rate):
    random_twist = Twist()

    steps = 0
    while not rospy.is_shutdown() and steps < duration:
        random_twist.linear.x = random.random() * 2 - 1
        random_twist.angular.z = (random.random() * 3.14159 * 2 - 3.14159) / scale
        publisher.publish(random_twist)
        rate.sleep()
        steps += 1

def move_turtle(shape, duration):
    # The Xth turtle accepts commands at: turtleX/cmd_vel
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)

    # Accepted format Twist (Vector3 linear, Vector3 angular)
    return {
        'square': _move_square,
        'triangle': _move_triangle,
        'circle': _move_circle,
        'random': _move_random,
    }.get(shape)(pub, duration, rate)

def handle_request(req):
    # Request: string shape, integer duration
    rospy.loginfo("Drawing %s for %ss." % (req.shape, req.duration))

    # Forward the commands to turtlebot
    move_turtle(req.shape, req.duration)

    # Prepare response that repeats the requested shape
    resp = CommandResponse()
    resp.response = req.shape
    rospy.loginfo("Returning %s" % resp)
    return resp

def move_server():
    rospy.init_node('move_server')
    random.seed(time())
    s = rospy.Service('move', Command, handle_request)
    rospy.loginfo("Prepared to move turle.")
    rospy.spin()

if __name__ == "__main__":
    move_server()
