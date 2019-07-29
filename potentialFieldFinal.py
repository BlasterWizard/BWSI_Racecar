#!/usr/bin/env python2
import numpy as np
import sys, math, random, copy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import rospy, copy, time
class PotentialField:
 SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic") #this gets changes to the direct topic when actually running it
 DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic") #^^^
 def __init__(self):
    self.data = None
    self.cmd = AckermannDriveStamped()
    #write your publishes and subscribers here; they should be the same as the wall follower's
    self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan,self.scan_callback, queue_size = 1)
    self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size = 1)
    #cartesian points -- to be filled (tuples)
    self.cartPointsX = [None for x in range(1081)] #1081 cause that's the LIDAR range on the car
    self.cartPointsY = [None for x in range(1081)]
    self.cartPoints = [None,None]
    #[speed, angle]
    self.finalVector = [0.5,0]
 def scan_callback(self,data): #called in the self.laser_sub subscriber
    #checks LIDAR data
    self.data = data
    self.drive_callback() #calls the drive function
 def drive_callback(self):
    #publishes drive commands
    self.finalVector = self.calcFinalVector() #self.calcFinalVector returns a list--a speed and angle value
    self.cmd.drive.speed = self.finalVector[0]
    self.cmd.drive.steering_angle = self.finalVector[1]
    self.drive_pub.publish(self.cmd)
 def convertPoints(self):
    #Convert all current LIDAR data to cartesian coordinates
    for number in range(1081):
       theta = (self.data.angle_min) + (number * self.data.angle_increment)
       theta = theta + (math.pi) #i want to flip all of my angle values in the opposite direction, because I want an equal but opposite vector.
       x = self.data.ranges[number]
       x = x * (math.cos(theta)) #this used to be a negative value but then i realized that with making the angle negative, i've done the job
       #x = 1.0/x i thought that taking the inverse would make it better, but it in fact makes the car yeet itself into walls
       y = self.data.ranges[number]
       y = y * (math.sin(theta))
       #y = 1.0/y
       self.cartPointsX[number] = x #I'm big dumb so I just made two lists here.
       self.cartPointsY[number] = y
    self.cartPoints[0] = self.cartPointsX
    self.cartPoints[1] = self.cartPointsY
    return self.cartPoints
 def calcFinalVector(self):
    #calculate the final driving speed and angle
    points = self.convertPoints()
    x = points[0] #self.cartPoints is a list with 2 elements: 1st is a list of X values, 2nd is a list of Y values.
    sumx = math.fsum(x) + 5
    y = points[1]
    sumy = math.fsum(y) + 5
    """print("left headlight: " + str((self.data.angle_min) + (58 * self.data.angle_increment)))
    print("right headlight: " + str((self.data.angle_min) + (40 * self.data.angle_increment)))
    print("coordinates are (" + str(sumx) + " ," + str(sumy) + ")")"""
    distance = math.sqrt(sumx**2 + sumy**2)
    driving_speed = distance * 0.25 #converts my distance vector to a velcocity vector
    theta = math.atan(sumy/sumx)
    print("speed: " + str(driving_speed) + " angle: " + str(theta)) #so I can easily check my values
    return [driving_speed,theta]
if __name__ == "__main__":
 rospy.init_node('potential_field')
 potential_field = PotentialField()
 rospy.spin()




Message Alex Towfigh
