#/usr/bin/env python

import rospy
import time
import numpy as np
import math
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan, Joy, Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String


DRIVE_TOPIC = "/drive"
SCAN_TOPIC = "/scan"
AR_TOPIC = "/ar_pose_marker"

class ARDrive(object):
    def __init__(self):
        print("init")
	rospy.init_node("ar")
        #initialize publishers and subscribers
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size = 1)
        self.scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.driveCallback)
        self.ar_sub = rospy.Subscriber(AR_TOPIC, AlvarMarkers, self.arCallback)
        self.sound_pub = rospy.Publisher("state", String, queue_size=1)

        #initialize cmd object
        self.cmd = AckermannDriveStamped()
        self.cmd.drive.speed = 0
        self.cmd.drive.steering_angle = 0

        self.state = [" ", " "]
        self.turnAngle = 10

    def driveCallback(self,data):
        '''LIDAR callback, sets drive commands'''
        #TODO: Set drive commands according to the current state
        
	print("running driveCallBack")
        print(self.state)
	for i in self.state:
            if  i == "Forward":

                print("Forward")
                self.cmd.drive.speed = 2
                self.sound_pub.publish("Forward")
                self.state[0] = " "
            elif i == "Backward":
                self.cmd.drive.speed = -2
                self.sound_pub.publish("Backward")
                self.state[0] = " "
            elif i == "Stop":
                self.cmd.drive.speed = 0
                self.sound_pub.publish("Stop")
                self.state[0] = " " 
            elif i == "Right":
                self.cmd.drive.steering_angle = -self.turnAngle * math.pi/180
                self.sound_pub.publish("Right")
                self.state[1] = " "
            elif i == "Left":
                self.cmd.drive.steering_angle = self.turnAngle * math.pi/180
                self.sound_pub.publish("Left")
                self.state[1] = " "
            elif i == "Straight":
                self.cmd.drive.steering_angle = 0
                self.sound_pub.publish("Straight")
                self.state[1] = " "


    def arCallback(self, tags):
        '''Callback when an AR tag is detected'''
        #TODO: Write your state changes here
        print("runing arCallback")
        print(tags.markers[0].id)

	if tags.markers[0].id != None:
            if tags.markers[0].id == 0:
                self.state[0] =  "Forward"
            elif tags.markers[0].id == 1:
                self.state[0] = "Backward"
            elif tags.markers[0].id == 2:
                self.state[0] = "Stop"
            elif tags.markers[0].id == 3:
                self.state[1] = "Right"
            elif tags.markers[0].id == 4:
                self.state[1] = "Left"
            elif tags.markers[0].id == 5:
                self.state[1] = "Straight"
        else:
            if tags.markers[1].id == 0:
                self.state[0] =  "Forward"
            elif tags.markers[1].id == 1:
                self.state[0] = "Backward"
            elif tags.markers[1].id == 2:
                self.state[0] = "Stop"
            elif tags.markers[1].id == 3:
                self.state[1] = "Right"
            elif tags.markers[1].id == 4:
                self.state[1] = "Left"
            elif tags.markers[1].id == 5:
                self.state[1] = "Straight"




def main():
    try:
        ic = ARDrive()
        rospy.Rate(100)
        while not rospy.is_shutdown():
            ic.drive_pub.publish(ic.cmd)
	   # ic.sound_pub.publish("Forward")
    except rospy.ROSInterruptException:
        exit()

if __name__ == "__main__":
    main()
