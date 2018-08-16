#!/usr/bin/env python

import math

import rospy

from robot_state import RobotState
from drive import Drive

class PositionController(object):
    targetX = 0
    targetY = 0

    pointQueue = []

    def __init__(self, forwardSpeed, tolerance):
        self.robotState = RobotState()
        self.drive = Drive()
        self.forwardSpeed = forwardSpeed
        self.TOLERANCE = tolerance

    def update(self):
        currentX = self.robotState.x
        currentY = self.robotState.y

        dist = math.sqrt((self.targetX - currentX)**2 + (self.targetY - currentY)**2)
        print currentX, currentY, dist
        if dist > self.TOLERANCE:
            targetHeading = math.atan2(self.targetY - currentY,
                    self.targetX - currentX)

            heading = self.robotState.clearSteer(self.robotState.heading,
                    targetHeading)

            self.drive.driveWithHeading(self.forwardSpeed, targetHeading, heading)
        else:
            self.nextPoint()

    def addPoint(self, x, y):
        self.pointQueue.append((x, y))

    def nextPoint(self):
        if len(self.pointQueue) > 0:
            pt = self.pointQueue.pop(0)
            self.targetX = pt[0]
            self.targetY = pt[1]

if __name__ == '__main__':
    pc = PositionController(0.1, 0.05)
    rospy.init_node('position_controller', anonymous=True)
    rate = rospy.Rate(10)

    pc.addPoint(0.25, 0)

    while not rospy.is_shutdown():
        pc.update()
        rate.sleep()
