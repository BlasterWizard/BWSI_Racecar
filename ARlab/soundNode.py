#!/usr/bin/env python

import time
import random
import math
import rospy
from std_msgs.msg import String
import os


#dictionary of states and their corresponding sound files
#TODO: fill this in
soundKeys = {"Forward":"Forward.wav" , "Backward": "Bback.wav", "Stop":"Stop.wav" , "Right": "Right.wav", "Left": "Left.wav" , "Straight": "Straight.wav" }


def callback(data):
    """
    Will recive a state via data that will correspond to a hard coded dict of audio files
    then will play file and then sleep for 2 seconds to avoid repeat messages
    """
    if data.data in soundKeys.keys():
        print("has data")
	os.system('/usr/bin/aplay ./pythonSounds/' + soundKeys[data.data])
        time.sleep(.1)

def main():
    '''initialize the node and subscriber'''
    rospy.init_node("listener")
    rospy.Subscriber("state", String, callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()
