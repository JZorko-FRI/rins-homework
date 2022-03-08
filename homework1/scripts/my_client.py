#!/usr/bin/env python


import sys
import rospy
from homework1.srv import Sum, SumResponse

def sum_array_client(array):
    rospy.wait_for_service('sum_array')
    try:
        sum_array = rospy.ServiceProxy('sum_array', Sum)
        resp = sum_array(array)
        return resp.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        _, *array = sys.argv
        array = [int(x) for x in array]
    else:
        print("%s [int]..." % sys.argv[0])
        sys.exit(1)

    print("Requesting sum of %s" % str(array))
    print("Returned sum is %s" % sum_array_client(array))
