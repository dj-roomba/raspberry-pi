#!/usr/bin/env python

import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

headingKp = 1

# Takes driving commands and publishes them to DriveSustainer
class Drive(object):
    def __init__(self):
        self.pub = rospy.Publisher('drive_cmd', Twist, queue_size=10)

    # simple drive with a forward speed and angular speed
    def drive(self, forward, turn):
        # make a new Twist with all 0 velocity
        twist = Twist()

        twist.linear.x = forward
        twist.angular.z = turn

        self.pub.publish(twist)

    # drive with a certain heading. implements a p control loop
    def driveWithHeading(self, forward, targetHeading, heading):
        turnSpeed = (targetHeading - heading) * headingKp
        self.drive(forward, turnSpeed)

# Publishes the most recent drive command to the roomba at 10Hz
# This allows the speed to be set once and have it held until another is set
class DriveSustainer(object):
    def __init__(self):
        rospy.Subscriber('drive_cmd', Twist, self.newCmd)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.lastCmd = Twist()

    def newCmd(self, cmd):
        self.lastCmd = cmd

    def sendCmd(self):
        self.pub.publish(self.lastCmd)

# When this file is run as a node, it runs DriveSustainer
# This should be done for most driving
if __name__ == '__main__':
      rospy.init_node('drive', anonymous=True)
      ds = DriveSustainer()
      rate = rospy.Rate(10)

      while not rospy.is_shutdown():
        ds.sendCmd()
        rate.sleep()
