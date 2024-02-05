#!/usr/bin/env python
import rospy
import std_msgs.msg

import os
import sys
sys.path.remove(os.path.dirname(__file__))

from receive_class_coordinates.msg import ClassCoordinates

def talker():
    pub = rospy.Publisher('information_1', ClassCoordinates, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    
    while not rospy.is_shutdown():
        my_msg = ClassCoordinates()

        my_msg.Header = std_msgs.msg.Header()
        my_msg.Header.stamp = rospy.Time.now()

        my_msg.classification = "human"
        my_msg.angle = 0.5
        my_msg.covariance_matrix = [0.0, 0.0, 0.0, 0.0]
        my_msg.centroide_distance = 2 # 2 m

        pub.publish(my_msg)
        rate.sleep()
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass