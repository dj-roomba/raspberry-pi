#!/usr/bin/env python

import math

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import tf

# Provides an easier way of accessing odometry data
class RobotState(object):
    x = 0
    y = 0
    xv = 0
    yv = 0
    heading = 0
    angularv = 0

    #def __init__(self):
    #    rospy.Subscriber('odom', Odometry, self.newOdometry)

    ## Called when the roomba publishes to odom (~10Hz)
    ## Need to check for None b/c that's what the roomba sends out when it
    ## starts up for some reason.
    #def newOdometry(self, odom):
    #    position = odom.pose.pose.position
    #    if position is not None:
    #        self.x = position.x
    #        self.y = position.y

    #    orientation = odom.pose.pose.orientation
    #    if orientation is not None:
    #        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
    #        roll, pitch, yaw = euler_from_quaternion(quat)
    #        self.heading = yaw

    #    linearVelocity = odom.twist.twist.linear
    #    if linearVelocity is not None:
    #        self.xv = linearVelocity.x
    #        self.yv = linearVelocity.y

    #    angularVelocity = odom.twist.twist.angular
    #    if angularVelocity is not None:
    #        self.angularv = angularVelocity

    def __init__(self):
        self.listener = tf.TransformListener()

    def updateTransform(self):
        try:
            trans, rot = self.listener.lookupTransform('/map', '/base_link',
                    rospy.Time(0))
        except Exception as e:
            return
        
        self.x = trans[0]
        self.y = trans[1]

    # Avoids the branch cuts in heading values when trying to run a control
    # loop (jumps from -pi to pi).
    # This isn't called by anything in RobotState, but is called by Drive
    def clearSteer(self, heading, target):
        if abs(target - heading) > math.pi:
            if target > 0:
                heading += 2 * math.pi
            else:
                heading -= 2 * math.pi
        return heading

