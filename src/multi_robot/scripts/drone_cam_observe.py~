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
        self.image_sub = rospy.Subscriber('drone01/downward_cam/camera/image', Image, self.image_callback)
        #self.image_sub_forward = rospy.Subscriber('drone01/front_cam/camera/image', Image, self.image_callback)
        self.pos_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.cmd_vel_pub01 = rospy.Publisher('drone01/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub02 = rospy.Publisher('drone02/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub03 = rospy.Publisher('drone03/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub04 = rospy.Publisher('drone04/cmd_vel',Twist, queue_size=10)
        self.twist = Twist()
    def image_callback(self, msg):
        
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8') #bgr8
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
            print("err_x: ", err_x, "err_y: ", err_y)
            
            self.twist.linear.x = 0.3#-float(err_y)/300.0
            #self.twist.linear.y = #-float(err_x)/300.0 + 0.5
            self.twist.angular.z = -float(err_y)/300 #-float(math.atan2(err_y,err_x))
            self.cmd_vel_pub01.publish(self.twist)
            #print(math.atan2(err_y,err_x))
            
        if M_ground['m00'] > 0:
            cx_ground = int(M_ground['m10']/M_ground['m00'])
            cy_ground = int(M_ground['m01']/M_ground['m00'])
            cv2.circle(image, (cx_ground, cy_ground), 10, (255,0,0), -1)
        if M_flood['m00'] > 0 and M_ground['m00'] > 0:
            cv2.line(image, (cx_flood, cy_flood), (cx_ground, cy_ground), (0,255,0), 1)
        cv2.imshow("mask_flood",mask_flood)
        cv2.imshow("mask_ground", mask_ground)
        cv2.imshow("output", image)
        cv2.waitKey(3)

    def callback(self, data):
        drone01 = data.pose[3]
        drone01_vel = data.twist[3]
        d01_posX = drone01.position.x
        d01_posY = drone01.position.y
        d01_posZ = drone01.position.z
        d01_velX = drone01_vel.linear.x
        d01_velY = drone01_vel.linear.y
        
        drone02 = data.pose[4]
        d02_posX = drone02.position.x
        d02_posY = drone02.position.y
        d02_posZ = drone02.position.z

        drone03 = data.pose[5]
        d03_posX = drone03.position.x
        d03_posY = drone03.position.y
        d03_posZ = drone03.position.z

        drone04 = data.pose[4]
        d04_posX = drone04.position.x
        d04_posY = drone04.position.y
        d04_posZ = drone04.position.z

        
        pub01 = self.cmd_vel_pub01
        vel_msg_z = Twist()

        pub02 = self.cmd_vel_pub02
        pub03 = self.cmd_vel_pub03
        pub04 = self.cmd_vel_pub04

        if d02_posZ >= 2:
            attract_x_d02, attract_y_d02 = calc_attractive_potential(d02_posX, d02_posY, d01_posX, d01_posY)
        else:
            attract_x_d02 = 0.0
            attract_y_d02 = 0.0

        if d03_posZ >= 2:
            attract_x_d03, attract_y_d03 = calc_attractive_potential(d03_posX, d03_posY, d01_posX, d01_posY)
        else:
            attract_x_d03 = 0.0
            attract_y_d03 = 0.0
            
        if d04_posZ >= 2:
            attract_x_d04, attract_y_d04 = calc_attractive_potential(d04_posX, d04_posY, d01_posX, d01_posY)
        else:
            attract_x_d04 = 0.0
            attract_y_d04 = 0.0

        new_velx_d02 = d01_velX + attract_x_d02
        new_vely_d02 = d01_velY + attract_y_d02
        new_velx_d03 = d01_velX + attract_x_d03
        new_vely_d03 = d01_velY + attract_y_d03
        new_velx_d04 = d01_velX + attract_x_d04
        new_vely_d04 = d01_velY + attract_y_d04

        calc_odom_d02 = (Vector3(new_velx_d02, new_vely_d02,-5*(d02_posZ-5)))
        calc_odom_d03 = (Vector3(new_velx_d03, new_vely_d03,-5*(d03_posZ-7)))
        calc_odom_d04 = (Vector3(new_velx_d04, new_vely_d04,-5*(d04_posZ-9)))

        pub02.publish(Twist(calc_odom_d02, Vector3(0,0,0)))
        pub03.publish(Twist(calc_odom_d03, Vector3(0,0,0)))
        pub04.publish(Twist(calc_odom_d04, Vector3(0,0,0)))
        
        calc_odom_d01 = (Vector3(d01_velX,d01_velY,-5*(d01_posZ-2)))
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
