#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'axes[0] : %f, data.axes[0])
    rospy.loginfo(rospy.get_caller_id() + 'axes[1] : %f, data.axes[1])
    rospy.loginfo(rospy.get_caller_id() + 'axes[2] : %f, data.axes[2])

def listener():
    rospy.init_node('tm_test', anonymous=True)
    rospy.Subscriber('Joy', Joy, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
