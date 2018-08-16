#!/usr/bin/env python

import serial
import math
import random

import rospy
import tf
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

import robot_state

if __name__ == "__main__":
    # initialize node and publisher
    rospy.init_node("plot_radiation", anonymous=True)
    pub = rospy.Publisher("radiation_cloud", PointCloud2, queue_size=1)

    # initialize tf listener
    listener = tf.TransformListener()

    # header to specify which frame this stuff is relative to
    # either map or odom should work
    header = Header(frame_id="/map")

    # data fields (it doesn't work without x, y, and z even though we really
    # only need x and y
    # cnt is the number of blips in the last .1s
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

    points = []

    # initialize serial connection
    ser = serial.Serial("/dev/ttyUSB2", 9600)

    while not rospy.is_shutdown():
        # get blip count from serial
        count = ser.readline()
        
        try:
            count = int(count)
        except ValueError:
            continue

        # get position of detector with tf
        try:
            trans, rot = listener.lookupTransform('/map', '/detector',
                rospy.Time(0))
        except Exception as e:
            # most likely the transform between map and detector isn't ready yet
            print(e)
            continue

        x = trans[0]
        y = trans[1]
        z = 0

        # graph each event as a point. they are displayed transparently so
        # mulitple should stack on top of one another
        for i in range(count):
            points.append([x, y, z])

        # republish the pt cloud if there is new data
        if count > 0:
            pointcloud = pc2.create_cloud(header, fields, points)
            pub.publish(pointcloud)
