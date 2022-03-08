#!/usr/bin/env python

from __future__ import print_function

import rospy
from homework1.srv import Sum, SumResponse

def handle_sum(req):
    rospy.loginfo("Recieved %s" % str(req.input))
    resp = SumResponse()
    resp.sum = sum(req.input)
    rospy.loginfo("Returning %d" % resp.sum)
    return resp

def sum_array_server():
    rospy.init_node('sum_array_server')
    s = rospy.Service('sum_array', Sum, handle_sum)
    print("Prepared to sum array.")
    rospy.spin()

if __name__ == "__main__":
    sum_array_server()
