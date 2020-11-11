#!/usr/bin/env python3

import rospy
import os
import math
import message_filters
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from geometry_msgs.msg import Twist, Vector3
import numpy as np

import cv2, cv_bridge
from sensor_msgs.msg import CompressedImage, CameraInfo

import configparser
import time


class Follower:
    def __init__(self):
        self.target = rospy.Subscriber('/unity_target', PoseStamped, self.target_callback)
        self.pub_target = rospy.Publisher('target', Float64MultiArray, queue_size=10)
        
    def target_callback(self, msg):
        target_pos = msg.pose.position
        print("target position: ", target_pos)
        target = np.array([target_pos.x, target_pos.y, target_pos.z])
        data2send = Float64MultiArray(data = target)
        self.pub_target.publish(data2send)
        rospy.Rate(10).sleep
    
    
           

def listener():
    MAX_DIST = 5
    rospy.init_node('target')
    follower = Follower()
    rospy.spin()

if __name__ == '__main__':
    start = time.time()
    
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
# END ALL
