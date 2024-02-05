#!/usr/bin/env python
import rospy

import os
import sys
sys.path.remove(os.path.dirname(__file__))

from receive_class_coordinates.msg import ClassCoordinates


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "angle: %s", data.angle) # cast to c


def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/information_1", ClassCoordinates, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    