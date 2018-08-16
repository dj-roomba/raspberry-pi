#!/usr/bin/env python

import random
import math

import rospy
from ca_msgs.msg import Bumper

import drive
import robot_state

class RandomDrive(object):
    def __init__(self):
        self.drive = drive.Drive()
        self.robotstate = robot_state.RobotState()
        self.turning = False
        self.targetHeading = 0
        self.headingThresh = 0.2
        self.forwardSpeed = 0.15
        self.turnSpeed = 0.5

        rospy.Subscriber('bumper', Bumper, self.whenBumped)

    def update(self):
        if self.turning:
            heading = self.robotstate.clearSteer(self.robotstate.heading, self.targetHeading)
            turnPower = math.copysign(self.turnSpeed, self.targetHeading - heading)
            self.drive.drive(0, turnPower)

            if abs(self.targetHeading - heading) < self.headingThresh:
                self.turning = False
        else:
            self.drive.driveWithHeading(self.forwardSpeed, self.targetHeading,
                    self.robotstate.clearSteer(self.robotstate.heading,
                        self.targetHeading))

    def whenBumped(self, bumper):
        obstacle = bumper.is_left_pressed or bumper.is_right_pressed or \
                bumper.is_light_left or bumper.is_light_front_left or \
                bumper.is_light_center_left or bumper.is_light_center_right or \
                bumper.is_light_front_right or bumper.is_light_right

        print obstacle

        if obstacle and not self.turning:
            heading = self.robotstate.heading
            self.targetHeading = random.uniform(heading +  3 * math.pi / 4,
                    heading +  5 * math.pi / 4)
            print heading, self.targetHeading
            self.turning = True

if __name__ == "__main__":
    rospy.init_node("random_drive", anonymous=False)
    rd = RandomDrive()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rd.update()
        rate.sleep()
