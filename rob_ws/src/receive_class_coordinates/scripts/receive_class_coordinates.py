#!/usr/bin/env python
import rospy

from receive_class_coordinates.msg import ClassCoordinates

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.angle) # cast to c


def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/information_1", String, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    