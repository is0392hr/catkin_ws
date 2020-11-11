#!/usr/bin/env python

import os
import sys
import tempfile
import csv
import tf
import rospy
import math
import message_filters
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import Twist, Vector3
import numpy as np

import cv2, cv_bridge
from sensor_msgs.msg import Image, CameraInfo

import configparser
import time 

MAX_DIST = 20

control = [0,0,0]
global control

def get_params():
    config = configparser.ConfigParser()
    config.read('index.txt')
    item_list = config.items('index')

    for item in item_list:
        key = item[0]
        value = item[1]

        if key == 'drone01':
            d1 = int(value)
        if key == 'drone02':
            d2 = int(value)    
        if key == 'drone03':
            d3 = int(value)
        if key == 'drone04':
            d4 = int(value)
        if key == 'drone05':
            d5 = int(value)
        if key == 'drone06':
            d6 = int(value)
        if key == 'drone07':
            d7 = int(value)
        if key == 'drone08':
            d8 = int(value)
        if key == 'drone09':
            d9 = int(value)
        if key == 'drone10':
            d10 = int(value)
        if key == 'drone11':
            d11 = int(value)
        if key == 'drone12':
            d12 = int(value)    
        if key == 'drone13':
            d13 = int(value)
        if key == 'drone14':
            d14 = int(value)
        if key == 'drone15':
            d15 = int(value)
        if key == 'drone16':
            d16 = int(value)
        if key == 'drone17':
            d17 = int(value)
        if key == 'drone18':
            d18 = int(value)
        if key == 'drone19':
            d19 = int(value)
        if key == 'drone20':
            d20 = int(value)
        if key == 'drone21':
            d21 = int(value)
        if key == 'drone22':
            d22 = int(value)    
        if key == 'drone23':
            d23 = int(value)
        if key == 'drone24':
            d24 = int(value)
        if key == 'drone25':
            d25 = int(value)
        if key == 'drone26':
            d26 = int(value)
        if key == 'drone27':
            d27 = int(value)
        if key == 'drone28':
            d28 = int(value)
        if key == 'drone29':
            d29 = int(value)
        if key == 'drone30':
            d30 = int(value)
        if key == 'drone31':
            d31 = int(value)
        if key == 'drone32':
            d32 = int(value)    
        if key == 'drone33':
            d33 = int(value)
        if key == 'drone34':
            d34 = int(value)
        if key == 'drone35':
            d35 = int(value)
        if key == 'drone36':
            d36 = int(value)
        if key == 'drone37':
            d37 = int(value)
        '''
        if key == 'drone08':
            d8 = int(value)
        if key == 'drone09':
            d9 = int(value)
        if key == 'drone10':
            d10 = int(value)
        '''

    params = [d1,d2,d3,d4,d5,d6,d7,d8,d9,d10,
            d11,d12,d13,d14,d15,d16,d17,d18,d19,d20,
            d21,d22,d23,d24,d25,d26,d27,d28,d29,d30,
            d31,d32,d33,d34,d35,d36,d37]
    return params


idx = get_params()
print(idx)

class Follower:
    def __init__(self):
        self.data = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.cmd_vel_pub1 = rospy.Publisher('drone01/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub2 = rospy.Publisher('drone02/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub3 = rospy.Publisher('drone03/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub4 = rospy.Publisher('drone04/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub5 = rospy.Publisher('drone05/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub6 = rospy.Publisher('drone06/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub7 = rospy.Publisher('drone07/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub8 = rospy.Publisher('drone08/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub9 = rospy.Publisher('drone09/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub10 = rospy.Publisher('drone10/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub11 = rospy.Publisher('drone11/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub12 = rospy.Publisher('drone12/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub13 = rospy.Publisher('drone13/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub14 = rospy.Publisher('drone14/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub15 = rospy.Publisher('drone15/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub16 = rospy.Publisher('drone16/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub17 = rospy.Publisher('drone17/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub18 = rospy.Publisher('drone18/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub19 = rospy.Publisher('drone19/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub20 = rospy.Publisher('drone20/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub21 = rospy.Publisher('drone21/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub22 = rospy.Publisher('drone22/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub23 = rospy.Publisher('drone23/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub24 = rospy.Publisher('drone24/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub25 = rospy.Publisher('drone25/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub26 = rospy.Publisher('drone26/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub27 = rospy.Publisher('drone27/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub28 = rospy.Publisher('drone28/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub29 = rospy.Publisher('drone29/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub30 = rospy.Publisher('drone30/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub31 = rospy.Publisher('drone31/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub32 = rospy.Publisher('drone32/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub33 = rospy.Publisher('drone33/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub34 = rospy.Publisher('drone34/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub35 = rospy.Publisher('drone35/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub36 = rospy.Publisher('drone36/cmd_vel',Twist, queue_size=10)
        self.cmd_vel_pub37 = rospy.Publisher('drone37/cmd_vel',Twist, queue_size=10)
        
        self.twist = Twist()
    

    def callback(self, data):

        drone01 = data.pose[idx[0]]
        drone01_vel = data.twist[idx[0]]
        d01_posX = drone01.position.x
        d01_posY = drone01.position.y
        d01_posZ = drone01.position.z
        d01_velX = drone01_vel.linear.x
        d01_velY = drone01_vel.linear.y
        
        drone02 = data.pose[idx[1]]
        drone02_vel = data.twist[idx[1]]
        d02_posX = drone02.position.x
        d02_posY = drone02.position.y
        d02_posZ = drone02.position.z
        d02_velX = drone02_vel.linear.x
        d02_velY = drone02_vel.linear.y

        drone03 = data.pose[idx[2]]
        drone03_vel = data.twist[idx[2]]
        d03_posX = drone03.position.x
        d03_posY = drone03.position.y
        d03_posZ = drone03.position.z
        d03_velX = drone03_vel.linear.x
        d03_velY = drone03_vel.linear.y

        drone04 = data.pose[idx[3]]
        drone04_vel = data.twist[idx[3]]
        d04_posX = drone04.position.x
        d04_posY = drone04.position.y
        d04_posZ = drone04.position.z
        d04_velX = drone04_vel.linear.x
        d04_velY = drone04_vel.linear.y

        drone05 = data.pose[idx[4]]
        drone05_vel = data.twist[idx[4]]
        d05_posX = drone05.position.x
        d05_posY = drone05.position.y
        d05_posZ = drone05.position.z
        d05_velX = drone05_vel.linear.x
        d05_velY = drone05_vel.linear.y

        drone06 = data.pose[idx[5]]
        drone06_vel = data.twist[idx[5]]
        d06_posX = drone06.position.x
        d06_posY = drone06.position.y
        d06_posZ = drone06.position.z
        d06_velX = drone06_vel.linear.x
        d06_velY = drone06_vel.linear.y

        drone07 = data.pose[idx[6]]
        drone07_vel = data.twist[idx[6]]
        d07_posX = drone07.position.x
        d07_posY = drone07.position.y
        d07_posZ = drone07.position.z
        d07_velX = drone07_vel.linear.x
        d07_velY = drone07_vel.linear.y

        drone08 = data.pose[idx[7]]
        drone08_vel = data.twist[idx[7]]
        d08_posX = drone08.position.x
        d08_posY = drone08.position.y
        d08_posZ = drone08.position.z
        d08_velX = drone08_vel.linear.x
        d08_velY = drone08_vel.linear.y

        drone09 = data.pose[idx[8]]
        drone09_vel = data.twist[idx[8]]
        d09_posX = drone09.position.x
        d09_posY = drone09.position.y
        d09_posZ = drone09.position.z
        d09_velX = drone09_vel.linear.x
        d09_velY = drone09_vel.linear.y

        drone10 = data.pose[idx[9]]
        drone10_vel = data.twist[idx[9]]
        d10_posX = drone10.position.x
        d10_posY = drone10.position.y
        d10_posZ = drone10.position.z
        d10_velX = drone10_vel.linear.x
        d10_velY = drone10_vel.linear.y

        drone11 = data.pose[idx[10]]
        drone11_vel = data.twist[idx[10]]
        d11_posX = drone11.position.x
        d11_posY = drone11.position.y
        d11_posZ = drone11.position.z
        d11_velX = drone11_vel.linear.x
        d11_velY = drone11_vel.linear.y
        
        drone12 = data.pose[idx[11]]
        drone12_vel = data.twist[idx[11]]
        d12_posX = drone12.position.x
        d12_posY = drone12.position.y
        d12_posZ = drone12.position.z
        d12_velX = drone12_vel.linear.x
        d12_velY = drone12_vel.linear.y

        drone13 = data.pose[idx[12]]
        drone13_vel = data.twist[idx[12]]
        d13_posX = drone13.position.x
        d13_posY = drone13.position.y
        d13_posZ = drone13.position.z
        d13_velX = drone13_vel.linear.x
        d13_velY = drone13_vel.linear.y

        drone14 = data.pose[idx[13]]
        drone14_vel = data.twist[idx[13]]
        d14_posX = drone14.position.x
        d14_posY = drone14.position.y
        d14_posZ = drone14.position.z
        d14_velX = drone14_vel.linear.x
        d14_velY = drone14_vel.linear.y

        drone15 = data.pose[idx[14]]
        drone15_vel = data.twist[idx[14]]
        d15_posX = drone15.position.x
        d15_posY = drone15.position.y
        d15_posZ = drone15.position.z
        d15_velX = drone15_vel.linear.x
        d15_velY = drone15_vel.linear.y

        drone16 = data.pose[idx[15]]
        drone16_vel = data.twist[idx[15]]
        d16_posX = drone16.position.x
        d16_posY = drone16.position.y
        d16_posZ = drone16.position.z
        d16_velX = drone16_vel.linear.x
        d16_velY = drone16_vel.linear.y

        drone17 = data.pose[idx[16]]
        drone17_vel = data.twist[idx[16]]
        d17_posX = drone17.position.x
        d17_posY = drone17.position.y
        d17_posZ = drone17.position.z
        d17_velX = drone17_vel.linear.x
        d17_velY = drone17_vel.linear.y

        drone18 = data.pose[idx[17]]
        drone18_vel = data.twist[idx[17]]
        d18_posX = drone18.position.x
        d18_posY = drone18.position.y
        d18_posZ = drone18.position.z
        d18_velX = drone18_vel.linear.x
        d18_velY = drone18_vel.linear.y

        drone19 = data.pose[idx[18]]
        drone19_vel = data.twist[idx[18]]
        d19_posX = drone19.position.x
        d19_posY = drone19.position.y
        d19_posZ = drone19.position.z
        d19_velX = drone19_vel.linear.x
        d19_velY = drone19_vel.linear.y

        drone20 = data.pose[idx[19]]
        drone20_vel = data.twist[idx[19]]
        d20_posX = drone20.position.x
        d20_posY = drone20.position.y
        d20_posZ = drone20.position.z
        d20_velX = drone20_vel.linear.x
        d20_velY = drone20_vel.linear.y

        drone21 = data.pose[idx[20]]
        drone21_vel = data.twist[idx[20]]
        d21_posX = drone21.position.x
        d21_posY = drone21.position.y
        d21_posZ = drone21.position.z
        d21_velX = drone21_vel.linear.x
        d21_velY = drone21_vel.linear.y
        
        drone22 = data.pose[idx[21]]
        drone22_vel = data.twist[idx[21]]
        d22_posX = drone22.position.x
        d22_posY = drone22.position.y
        d22_posZ = drone22.position.z
        d22_velX = drone22_vel.linear.x
        d22_velY = drone22_vel.linear.y

        drone23 = data.pose[idx[22]]
        drone23_vel = data.twist[idx[22]]
        d23_posX = drone23.position.x
        d23_posY = drone23.position.y
        d23_posZ = drone23.position.z
        d23_velX = drone23_vel.linear.x
        d23_velY = drone23_vel.linear.y

        drone24 = data.pose[idx[23]]
        drone24_vel = data.twist[idx[23]]
        d24_posX = drone24.position.x
        d24_posY = drone24.position.y
        d24_posZ = drone24.position.z
        d24_velX = drone24_vel.linear.x
        d24_velY = drone24_vel.linear.y

        drone25 = data.pose[idx[24]]
        drone25_vel = data.twist[idx[24]]
        d25_posX = drone25.position.x
        d25_posY = drone25.position.y
        d25_posZ = drone25.position.z
        d25_velX = drone25_vel.linear.x
        d25_velY = drone25_vel.linear.y

        drone26 = data.pose[idx[25]]
        drone26_vel = data.twist[idx[25]]
        d26_posX = drone26.position.x
        d26_posY = drone26.position.y
        d26_posZ = drone26.position.z
        d26_velX = drone26_vel.linear.x
        d26_velY = drone26_vel.linear.y

        drone27 = data.pose[idx[26]]
        drone27_vel = data.twist[idx[26]]
        d27_posX = drone27.position.x
        d27_posY = drone27.position.y
        d27_posZ = drone27.position.z
        d27_velX = drone27_vel.linear.x
        d27_velY = drone27_vel.linear.y

        drone28 = data.pose[idx[27]]
        drone28_vel = data.twist[idx[27]]
        d28_posX = drone28.position.x
        d28_posY = drone28.position.y
        d28_posZ = drone28.position.z
        d28_velX = drone28_vel.linear.x
        d28_velY = drone28_vel.linear.y

        drone29 = data.pose[idx[28]]
        drone29_vel = data.twist[idx[28]]
        d29_posX = drone29.position.x
        d29_posY = drone29.position.y
        d29_posZ = drone29.position.z
        d29_velX = drone29_vel.linear.x
        d29_velY = drone29_vel.linear.y

        drone30 = data.pose[idx[29]]
        drone30_vel = data.twist[idx[29]]
        d30_posX = drone30.position.x
        d30_posY = drone30.position.y
        d30_posZ = drone30.position.z
        d30_velX = drone30_vel.linear.x
        d30_velY = drone30_vel.linear.y

        drone31 = data.pose[idx[30]]
        drone31_vel = data.twist[idx[30]]
        d31_posX = drone31.position.x
        d31_posY = drone31.position.y
        d31_posZ = drone31.position.z
        d31_velX = drone31_vel.linear.x
        d31_velY = drone31_vel.linear.y
        
        drone32 = data.pose[idx[31]]
        drone32_vel = data.twist[idx[31]]
        d32_posX = drone32.position.x
        d32_posY = drone32.position.y
        d32_posZ = drone32.position.z
        d32_velX = drone32_vel.linear.x
        d32_velY = drone32_vel.linear.y

        drone33 = data.pose[idx[32]]
        drone33_vel = data.twist[idx[32]]
        d33_posX = drone33.position.x
        d33_posY = drone33.position.y
        d33_posZ = drone33.position.z
        d33_velX = drone33_vel.linear.x
        d33_velY = drone33_vel.linear.y

        drone34 = data.pose[idx[33]]
        drone34_vel = data.twist[idx[33]]
        d34_posX = drone34.position.x
        d34_posY = drone34.position.y
        d34_posZ = drone34.position.z
        d34_velX = drone34_vel.linear.x
        d34_velY = drone34_vel.linear.y

        drone35 = data.pose[idx[34]]
        drone35_vel = data.twist[idx[34]]
        d35_posX = drone35.position.x
        d35_posY = drone35.position.y
        d35_posZ = drone35.position.z
        d35_velX = drone35_vel.linear.x
        d35_velY = drone35_vel.linear.y

        drone36 = data.pose[idx[35]]
        drone36_vel = data.twist[idx[35]]
        d36_posX = drone36.position.x
        d36_posY = drone36.position.y
        d36_posZ = drone36.position.z
        d36_velX = drone36_vel.linear.x
        d36_velY = drone36_vel.linear.y

        drone37 = data.pose[idx[36]]
        drone37_vel = data.twist[idx[36]]
        d37_posX = drone37.position.x
        d37_posY = drone37.position.y
        d37_posZ = drone37.position.z
        d37_velX = drone37_vel.linear.x
        d37_velY = drone37_vel.linear.y

        


        #print(control,d04_posZ)
        ####################    
        ## POSITION BASED ##
        ####################
        
        pub1 = self.cmd_vel_pub1
        calc_odom1 = (Vector3(d01_velX,d01_velY,-0.8*(d01_posZ-3)))
        
        pub2 = self.cmd_vel_pub2
        calc_odom2 = (Vector3(d02_velX,d02_velY,-0.8*(d02_posZ-3)))
        
        pub3 = self.cmd_vel_pub3
        calc_odom3 = (Vector3(d03_velX,d03_velY,-0.8*(d03_posZ-3)))
        
        pub4 = self.cmd_vel_pub4
        calc_odom4 = (Vector3(d04_velX,d04_velY,-0.8*(d04_posZ-3)))
        
        pub5 = self.cmd_vel_pub5
        calc_odom5 = (Vector3(d05_velX,d05_velY,-0.8*(d05_posZ-3)))
        
        pub6 = self.cmd_vel_pub6
        calc_odom6 = (Vector3(d06_velX,d06_velY,-0.8*(d06_posZ-3)))
        
        pub7 = self.cmd_vel_pub7
        calc_odom7 = (Vector3(d07_velX,d07_velY,-0.8*(d07_posZ-3)))
        
        pub8 = self.cmd_vel_pub8
        calc_odom8 = (Vector3(d08_velX,d08_velY,-0.8*(d08_posZ-3)))
        
        pub9 = self.cmd_vel_pub9
        calc_odom9 = (Vector3(d09_velX,d09_velY,-0.8*(d09_posZ-3)))
        
        pub10 = self.cmd_vel_pub10
        calc_odom10 = (Vector3(d10_velX,d10_velY,-0.8*(d10_posZ-3)))
        
        pub11 = self.cmd_vel_pub11
        calc_odom11 = (Vector3(d11_velX,d11_velY,-0.8*(d11_posZ-3)))
        
        pub12 = self.cmd_vel_pub12
        calc_odom12 = (Vector3(d12_velX,d12_velY,-0.8*(d12_posZ-3)))
        
        pub13 = self.cmd_vel_pub13
        calc_odom13 = (Vector3(d13_velX,d13_velY,-0.8*(d13_posZ-3)))
        
        pub14 = self.cmd_vel_pub14
        calc_odom14 = (Vector3(d14_velX,d14_velY,-0.8*(d14_posZ-3)))
        
        pub15 = self.cmd_vel_pub15
        calc_odom15 = (Vector3(d15_velX,d15_velY,-0.8*(d15_posZ-3)))
        
        pub16 = self.cmd_vel_pub16
        calc_odom16 = (Vector3(d16_velX,d16_velY,-0.8*(d16_posZ-3)))
        
        pub17 = self.cmd_vel_pub17
        calc_odom17 = (Vector3(d17_velX,d17_velY,-0.8*(d17_posZ-3)))
        
        pub18 = self.cmd_vel_pub18
        calc_odom18 = (Vector3(d18_velX,d18_velY,-0.8*(d18_posZ-3)))
        
        pub19 = self.cmd_vel_pub19
        calc_odom19 = (Vector3(d19_velX,d19_velY,-0.8*(d19_posZ-3)))
        
        pub20 = self.cmd_vel_pub20
        calc_odom20 = (Vector3(d20_velX,d20_velY,-0.8*(d20_posZ-3)))
        
        pub21 = self.cmd_vel_pub21
        calc_odom21 = (Vector3(d21_velX,d21_velY,-0.8*(d21_posZ-3)))
        
        pub22 = self.cmd_vel_pub22
        calc_odom22 = (Vector3(d22_velX,d22_velY,-0.8*(d22_posZ-3)))
        
        pub23 = self.cmd_vel_pub23
        calc_odom23 = (Vector3(d23_velX,d23_velY,-0.8*(d23_posZ-3)))
        
        pub24 = self.cmd_vel_pub24
        calc_odom24 = (Vector3(d24_velX,d24_velY,-0.8*(d24_posZ-3)))
        
        pub25 = self.cmd_vel_pub25
        calc_odom25 = (Vector3(d25_velX,d25_velY,-0.8*(d25_posZ-3)))
        
        pub26 = self.cmd_vel_pub26
        calc_odom26 = (Vector3(d26_velX,d26_velY,-0.8*(d26_posZ-3)))
        
        pub27 = self.cmd_vel_pub27
        calc_odom27 = (Vector3(d27_velX,d27_velY,-0.8*(d27_posZ-3)))
        
        pub28 = self.cmd_vel_pub28
        calc_odom28 = (Vector3(d28_velX,d28_velY,-0.8*(d28_posZ-3)))
        
        pub29 = self.cmd_vel_pub29
        calc_odom29 = (Vector3(d29_velX,d29_velY,-0.8*(d29_posZ-3)))
        
        pub30 = self.cmd_vel_pub30
        calc_odom30 = (Vector3(d30_velX,d30_velY,-0.8*(d30_posZ-3)))
        
        pub31 = self.cmd_vel_pub31
        calc_odom31 = (Vector3(d31_velX,d31_velY,-0.8*(d31_posZ-3)))
        
        pub32 = self.cmd_vel_pub32
        calc_odom32 = (Vector3(d32_velX,d32_velY,-0.8*(d32_posZ-3)))
        
        pub33 = self.cmd_vel_pub33
        calc_odom33 = (Vector3(d33_velX,d33_velY,-0.8*(d33_posZ-3)))
        
        pub34 = self.cmd_vel_pub34
        calc_odom34 = (Vector3(d34_velX,d34_velY,-0.8*(d34_posZ-3)))
        
        pub35 = self.cmd_vel_pub35
        calc_odom35 = (Vector3(d35_velX,d35_velY,-0.8*(d35_posZ-3)))
        
        pub36 = self.cmd_vel_pub36
        calc_odom36 = (Vector3(d36_velX,d36_velY,-0.8*(d36_posZ-3)))
        
        pub37 = self.cmd_vel_pub37
        calc_odom37 = (Vector3(d37_velX,d37_velY,-0.8*(d37_posZ-3)))
    

        #################
        ##   publish   ##
        #################
        pub1.publish(Twist(calc_odom1, Vector3(0,0,0)))
        pub2.publish(Twist(calc_odom2, Vector3(0,0,0)))
        pub3.publish(Twist(calc_odom3, Vector3(0,0,0)))
        pub4.publish(Twist(calc_odom4, Vector3(0,0,0)))
        pub5.publish(Twist(calc_odom5, Vector3(0,0,0)))
        pub6.publish(Twist(calc_odom6, Vector3(0,0,0)))
        pub7.publish(Twist(calc_odom7, Vector3(0,0,0)))
        pub8.publish(Twist(calc_odom8, Vector3(0,0,0)))
        pub9.publish(Twist(calc_odom9, Vector3(0,0,0)))
        pub10.publish(Twist(calc_odom10, Vector3(0,0,0)))
        pub11.publish(Twist(calc_odom11, Vector3(0,0,0)))
        pub12.publish(Twist(calc_odom12, Vector3(0,0,0)))
        pub13.publish(Twist(calc_odom13, Vector3(0,0,0)))
        pub14.publish(Twist(calc_odom14, Vector3(0,0,0)))
        pub15.publish(Twist(calc_odom15, Vector3(0,0,0)))
        pub16.publish(Twist(calc_odom16, Vector3(0,0,0)))
        pub17.publish(Twist(calc_odom17, Vector3(0,0,0)))
        pub18.publish(Twist(calc_odom18, Vector3(0,0,0)))
        pub19.publish(Twist(calc_odom19, Vector3(0,0,0)))
        pub20.publish(Twist(calc_odom20, Vector3(0,0,0)))
        pub21.publish(Twist(calc_odom21, Vector3(0,0,0)))
        pub22.publish(Twist(calc_odom22, Vector3(0,0,0)))
        pub23.publish(Twist(calc_odom23, Vector3(0,0,0)))
        pub24.publish(Twist(calc_odom24, Vector3(0,0,0)))
        pub25.publish(Twist(calc_odom25, Vector3(0,0,0)))
        pub26.publish(Twist(calc_odom26, Vector3(0,0,0)))
        pub27.publish(Twist(calc_odom27, Vector3(0,0,0)))
        pub28.publish(Twist(calc_odom28, Vector3(0,0,0)))
        pub29.publish(Twist(calc_odom29, Vector3(0,0,0)))
        pub30.publish(Twist(calc_odom30, Vector3(0,0,0)))
        pub31.publish(Twist(calc_odom31, Vector3(0,0,0)))
        pub32.publish(Twist(calc_odom32, Vector3(0,0,0)))
        pub33.publish(Twist(calc_odom33, Vector3(0,0,0)))
        pub34.publish(Twist(calc_odom34, Vector3(0,0,0)))
        pub35.publish(Twist(calc_odom35, Vector3(0,0,0)))
        pub36.publish(Twist(calc_odom36, Vector3(0,0,0)))
        pub37.publish(Twist(calc_odom37, Vector3(0,0,0)))



        #print('position control now')
        rospy.Rate(1).sleep
           

    
def listener():
    rospy.init_node('d_100')
    follower = Follower()
    rospy.spin()

if __name__ == '__main__':
    start = time.time()
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
# END ALL
