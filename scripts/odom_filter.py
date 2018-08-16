#!/usr/bin/env python

import numpy as np

import rospy
from nav_msgs.msg import Odometry

pub = None

def getOdom(odom):
    print("got odom")
    odom.pose.covariance = tuple([0] * 36)
    odom.twist.covariance = tuple([0] * 36)

    if pub is not None:
        pub.publish(odom)

if __name__ == '__main__':
    rospy.init_node('odomFilter', anonymous=True)
    rospy.Subscriber('odom', Odometry, getOdom)
    pub = rospy.Publisher('filtered_odom', Odometry, queue_size=10)
    rospy.spin()
