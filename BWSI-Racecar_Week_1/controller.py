#!/usr/bin/env python2

import numpy as np
import sys, math, random, copy
import rospy, copy, time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class PotentialField:
    SCAN_TOPIC = "/scan"
    DRIVE_TOPIC = "/drive"

    def __init__(self):
        self.data = None
        self.cmd = AckermannDriveStamped()
        print "Controller standing by"
        #write your publishers and subscribers here; they should be the same as the wall follower's
        self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        
        #[speed, angle]
        self.finalVector = [0, 0]

    def scan_callback(self, data):
        '''Checks LIDAR data'''
        #cartesian points -- to be filled (tuples)
        self.cartPoints = [None for x in range(len(data.ranges))]
        self.data = data.ranges
        self.drive_callback()

    def drive_callback(self):
        '''Publishes drive commands'''
        #make sure to publish cmd here
        self.convertPoints(self.data)
        self.calcFinalVector(self.cartPoints)
        self.cmd.drive.speed = self.finalVector[0]
        self.cmd.drive.steering_angle = self.finalVector[1]
        self.drive_pub.publish(self.cmd)

    def convertPoints(self, points):
        '''Convert all current LIDAR data to cartesian coordinates'''
        for i in range(len(points)):
            self.cartPoints[i] = (-1/points[i]*math.cos(float(i*4.71/(len(points)-1)-2.355)), -1/points[i]*math.sin(float(i*4.71/(len(points)-1)-2.355)))

    def calcFinalVector(self, points):
        '''Calculate the final driving speed and angle'''
        sumx = sum(i[0] for i in self.cartPoints)
        sumx += 450
        sumy = sum(i[1] for i in self.cartPoints)
        self.finalVector[0] = max(10, math.sqrt((sumy**2) + (sumx**2)) / 50)
        self.finalVector[1] = math.atan2(sumy, sumx)/2.18

if __name__ == "__main__":
    rospy.init_node('potential_field')
    potential_field = PotentialField()
    rospy.spin()
