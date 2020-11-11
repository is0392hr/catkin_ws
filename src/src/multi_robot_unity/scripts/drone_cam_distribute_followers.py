#!/usr/bin/env python

import os
import sys
import tempfile
import csv
import tf
import rospy
import math
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import Twist, Vector3
import numpy as np

import cv2, cv_bridge
from sensor_msgs.msg import Image, CameraInfo


def calc_attractive_potential(x, y, tx, ty):
    dist = 0.5*math.sqrt((x-tx)**2 + (y-ty)**2)
    if(dist<=3):
        return 0, 0
    else: 
        return dist*(tx-x), dist*(ty-y)

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('drone02/front_cam/camera/image', Image, self.image_callback)
        self.pos_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.cmd_vel_pub02 = rospy.Publisher('drone02/cmd_vel',Twist, queue_size=10)
        self.twist = Twist()
        
    

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8') #bgr8
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_drone = np.array([20,100, 100])#yellow #np.array([0, 100, 100])
        upper_drone = np.array([80, 255, 255])#yellow #np.array([100, 255, 255])
        mask_drone = cv2.inRange(hsv, lower_drone, upper_drone)
        
        h, w, d = image.shape
        
        search_top = 0
        search_bot = h
        mask_drone[0:search_top, 0:w] = 0
        mask_drone[search_bot:h, 0:w] = 0
        M_drone = cv2.moments(mask_drone)
        
    
        if M_drone['m00'] > 0:
            cx_drone = int(M_drone['m10']/M_drone['m00'])
            cy_drone = int(M_drone['m01']/M_drone['m00'])
            cv2.circle(image, (cx_drone, cy_drone), 10, (0,0,255), -1)

            # BEGIN CONTROL
            err_x = cx_drone - w/2
            err_y = cy_drone - h/2
            print("err_x: ", err_x, "err_y: ", err_y)

            if cx_drone < w/2:
                self.twist.linear.x = -float(err_y)/5
            else:
                self.twist.linear.x = -float(err_y)/20.0    #-float(err_y)/300.0 + 1.0
                # self.twist.linear.y =  #-float(err_x)/85.0
                self.twist.angular.z = -float(err_x)/20.0
            self.cmd_vel_pub02.publish(self.twist)
            # END CONTROL

        else:
            self.twist.linear.x = 0
            self.twist.angular.z = -1.0
            self.cmd_vel_pub02.publish(self.twist)
            
        
        cv2.imshow("output_front", image)
        cv2.imshow("mask_drone", mask_drone)
        
        cv2.waitKey(3)

    def callback(self, data):

        drone02 = data.pose[10]
        drone02_vel = data.twist[10]
        d02_posX = drone02.position.x
        d02_posY = drone02.position.y
        d02_posZ = drone02.position.z
        d02_velX = drone02_vel.linear.x
        d02_velY = drone02_vel.linear.y
        
        pub02 = self.cmd_vel_pub02
        vel_msg_z = Twist()

        calc_odom_d02 = (Vector3(d02_velX,d02_velY,-5*(d02_posZ-4)))
        pub02.publish(Twist(calc_odom_d02, Vector3(0,0,0)))
        
        rospy.Rate(10).sleep

def listener():
    rospy.init_node('follower')
    follower = Follower()
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
# END ALL
