#!/usr/bin/env python
import rospy
import std_msgs.msg

import os
import sys
sys.path.remove(os.path.dirname(__file__))

from multi_obstacles_tracker_msgs.msg import CameraDetectionStampedArray
from multi_obstacles_tracker_msgs.msg import CameraDetectionStamped

def talker():
    pub = rospy.Publisher('information_1', CameraDetectionStampedArray, queue_size=1)
    rospy.init_node('test_CameraDetectionsStamped', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        my_msg_array = CameraDetectionStampedArray()
        my_msg = CameraDetectionStamped()

        my_msg.header = std_msgs.msg.Header()
        my_msg.header.stamp = rospy.get_rostime()

        my_msg_array.header = my_msg.header

        my_msg.classification = "person"
        my_msg.coordinates[0] = 1.40
        my_msg.coordinates[1] = 160.5
        my_msg.covariance = [0.37, 0.61, 0.61, 1.87]
        
        my_msg_array.detections.append(my_msg)

        my_msg2 = CameraDetectionStamped()
        my_msg2.classification = "person"
        my_msg2.coordinates[0] = 0.58
        my_msg2.coordinates[1] = 290.42
        my_msg2.covariance = [0.20, 0.06, 0.606, 0.05]
        
        my_msg_array.detections.append(my_msg2)

        pub.publish(my_msg_array)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass