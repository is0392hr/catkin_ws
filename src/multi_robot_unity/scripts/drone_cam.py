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


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('drone01/downward_cam/camera/image', Image, self.image_callback)
        self.pos_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.cmd_vel_pub = rospy.Publisher('drone01/cmd_vel',Twist, queue_size=10)
        self.twist = Twist()
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8') #bgr8
        #color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_flood = np.array([0, 100, 100])
        upper_flood = np.array([100, 255, 255])
        lower_ground = np.array([0, 0, 0])
        upper_ground = np.array([100, 100, 100])
        mask_flood = cv2.inRange(hsv, lower_flood, upper_flood)
        mask_ground = cv2.inRange(hsv, lower_ground, upper_ground)
        
        h, w, d = image.shape
        search_top = 0#3*h/4
        search_bot = h#3*h/4 + 20
        mask_flood[0:search_top, 0:w] = 0
        mask_flood[search_bot:h, 0:w] = 0
        mask_ground[0:search_top, 0:w] = 0
        mask_ground[search_bot:h, 0:w] = 0
        M_flood = cv2.moments(mask_flood)
        M_ground = cv2.moments(mask_ground)
        
        if M_flood['m00'] > 0:
            cx_flood = int(M_flood['m10']/M_flood['m00'])
            cy_flood = int(M_flood['m01']/M_flood['m00'])
            cv2.circle(image, (cx_flood, cy_flood), 10, (0,0,255), -1)
            # BEGIN CONTROL
            err_y = cx_flood - w/2
            err_x = cy_flood - h/2
            if abs(math.atan2(err_y,err_x)) < 2:
                print("speed up!!")
                self.twist.linear.x = -float(err_y)/100
                self.twist.linear.y = -float(err_x)/100
            else:
                self.twist.linear.x = -float(err_y)/300 + 0.3
                self.twist.linear.y = -float(err_x)/100
                
            if math.atan2(err_y,err_x) >= 0:
                self.twist.angular.z = -float(math.atan2(err_y,err_x))*1.1
            else:
                self.twist.angular.z = float(math.atan2(err_y,err_x))*1.1
                
                self.cmd_vel_pub.publish(self.twist)
            print(math.atan2(err_y,err_x))
        if M_ground['m00'] > 0:
            cx_ground = int(M_ground['m10']/M_ground['m00'])
            cy_ground = int(M_ground['m01']/M_ground['m00'])
            cv2.circle(image, (cx_ground, cy_ground), 10, (255,0,0), -1)
       
        cv2.imshow("mask_flood",mask_flood)
        cv2.imshow("mask_ground", mask_ground)
        cv2.imshow("output", image)
        cv2.waitKey(3)

    def callback(self, data):
        drone01 = data.pose[1]
        d01_posX = drone01.position.x
        d01_posY = drone01.position.y
        d01_posZ = drone01.position.z

        
        pub01 = self.cmd_vel_pub
        vel_msg_z = Twist()
        distance = 10
        speed = 0.8
        
        calc_odom_d01 = (Vector3(self.twist.linear.x,self.twist.linear.y,-5*(d01_posZ-2)))
        pub01.publish(Twist(calc_odom_d01, Vector3(0,0,0)))
        
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
