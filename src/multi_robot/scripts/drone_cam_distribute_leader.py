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

Mx = 0.00
M1x = 0.00
goal_x = 480/2

def calc_attractive_potential(x, y, tx, ty):
    dist = 0.5*math.sqrt((x-tx)**2 + (y-ty)**2)
    if(dist<=3):
        return 0, 0
    else: 
        return dist*(tx-x), dist*(ty-y)

class Leader:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('drone01/downward_cam/camera/image', Image, self.image_callback)
        self.pos_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.cmd_vel_pub01 = rospy.Publisher('drone01/cmd_vel',Twist, queue_size=10)
        self.twist = Twist()
        
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8') #bgr8
        image = image[:, 30:image.shape[1]-30, :]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_flood = np.array([0, 100, 100])#np.array([100, 100, 0]) #yellow #np.array([0, 100, 100])
        upper_flood = np.array([150, 255, 255])#np.array([255, 255, 120]) #yellow #np.array([100, 255, 255])
        lower_ground = np.array([0, 0, 0])
        upper_ground = np.array([110, 110, 110])
        mask_flood = cv2.inRange(hsv, lower_flood, upper_flood)
        mask_ground = cv2.inRange(hsv, lower_ground, upper_ground)

        h, w, d = image.shape
        search_top = 0#3*h/4
        search_bot = h#3*h/4 + 20
        mask_flood[0:search_top, 30:w-30] = 0
        mask_flood[search_bot:h, 30:w-30] = 0
        mask_ground[0:search_top, 30:w-30] = 0
        mask_ground[search_bot:h, 30:w-30] = 0
        M_flood = cv2.moments(mask_flood)
        M_ground = cv2.moments(mask_ground)
        
        if M_flood['m00'] > 0:
            cx_flood = int(M_flood['m10']/M_flood['m00'])
            cy_flood = int(M_flood['m01']/M_flood['m00'])
            cv2.circle(image, (cx_flood, cy_flood), 10, (0,0,255), -1)
        
        if M_ground['m00'] > 0:
            cx_ground = int(M_ground['m10']/M_ground['m00'])
            cy_ground = int(M_ground['m01']/M_ground['m00'])
            cv2.circle(image, (cx_ground, cy_ground), 10, (255,0,0), -1)
        if M_flood['m00'] > 0 and M_ground['m00'] > 0:
            cx_flood = int(M_flood['m10']/M_flood['m00'])
            cy_flood = int(M_flood['m01']/M_flood['m00'])
            cv2.circle(image, (cx_flood, cy_flood), 10, (0,0,255), -1)
            cx_ground = int(M_ground['m10']/M_ground['m00'])
            cy_ground = int(M_ground['m01']/M_ground['m00'])
            cv2.circle(image, (cx_ground, cy_ground), 10, (255,0,0), -1)

            cx_middle = min(cx_flood, cx_ground) + int((max(cx_flood, cx_ground)-min(cx_flood, cx_ground))/2)
            cy_middle = min(cy_flood, cy_ground) + int((max(cy_flood, cy_ground)-min(cy_flood, cy_ground))/2)
            cv2.circle(image, (cx_middle, cy_middle), 10, (0,0,0), -1)
            cv2.line(image, (cx_flood, cy_flood), (cx_ground, cy_ground), (0,255,0), 1)

            
            # BEGIN CONTROL
            err_x = cx_middle - w/2
            err_y = cy_middle - h/2
            
            print("err_x: ", err_x, "err_y: ", err_y)
            
            self.twist.linear.x = -float(err_y)/300.0 + 1.0
            self.twist.linear.y =  -float(err_x)/85.0
            self.twist.angular.z = -float(math.atan2((cy_flood - cy_ground),(cx_flood - cx_ground)))*1.8
            self.cmd_vel_pub01.publish(self.twist)
            #print(math.atan2(err_y,err_x))
            
        cv2.imshow("mask_flood",mask_flood)
        cv2.imshow("mask_ground", mask_ground)
        cv2.imshow("output", image)
        cv2.waitKey(3)

    def callback(self, data):
        drone01 = data.pose[13]
        drone01_vel = data.twist[13]
        d01_posX = drone01.position.x
        d01_posY = drone01.position.y
        d01_posZ = drone01.position.z
        d01_velX = drone01_vel.linear.x
        d01_velY = drone01_vel.linear.y
        

        
        pub01 = self.cmd_vel_pub01
        vel_msg_z = Twist()


        
        calc_odom_d01 = (Vector3(d01_velX,d01_velY,-5*(d01_posZ-2)))
        pub01.publish(Twist(calc_odom_d01, Vector3(0,0,0)))
        
        rospy.Rate(10).sleep

def listener():
    rospy.init_node('leader')
    leader = Leader()
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
# END ALL
