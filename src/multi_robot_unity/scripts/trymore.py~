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

def calc_attractive_potential(x, y, tx, ty):
    dist = 0.5*math.sqrt((x-tx)**2 + (y-ty)**2)
    if(dist<=3):
        return 0, 0
    else: 
        return dist*(tx-x), dist*(ty-y)

def calc_repulsive_potential(x1, y1, x2, y2):
    dist = math.sqrt((x1-x2)**2 + (y1-y2)**2)
    if(dist<=2.5):
        force_x = 3*(math.sqrt((x1-x2)**2))-7.5 #dist*gain
        force_y = 3*(math.sqrt((y1-y2)**2))-7.5
        if(x2>x1):
	    force_x = -force_x
        if(y2>y1):
	    force_y = -force_y
    else:
	force_x=0.0 
	force_y=0.0

    return force_x, force_y

def callback(data):
    pub01 = rospy.Publisher('drone01/cmd_vel',Twist,queue_size=10)
    pub02 = rospy.Publisher('drone02/cmd_vel',Twist,queue_size=10)
    pub03 = rospy.Publisher('drone03/cmd_vel',Twist,queue_size=10)
    pub04 = rospy.Publisher('drone04/cmd_vel',Twist,queue_size=10)
    pub05 = rospy.Publisher('drone05/cmd_vel',Twist,queue_size=10)
    pub06 = rospy.Publisher('drone06/cmd_vel',Twist,queue_size=10)
    pub1 = rospy.Publisher('drone1/cmd_vel',Twist,queue_size=10)
    pub2 = rospy.Publisher('drone2/cmd_vel',Twist,queue_size=10)
    pub3 = rospy.Publisher('drone3/cmd_vel',Twist,queue_size=10)

        
    drone01 = data.pose[2]
    drone02 = data.pose[1]
    drone03 = data.pose[4]
    drone04 = data.pose[3]
    drone05 = data.pose[5]
    drone06 = data.pose[7]
    drone1 = data.pose[6]
    drone2 = data.pose[8]
    drone3 = data.pose[9]
    
    d01_vel = data.twist[2]
    d01_velLin_x = d01_vel.linear.x
    d01_velLin_y = d01_vel.linear.y

    d01_posX = drone01.position.x
    d01_posY = drone01.position.y
    d01_posZ = drone01.position.z

    d02_vel = data.twist[1]
    d02_velLin_x = d02_vel.linear.x
    d02_velLin_y = d02_vel.linear.y

    d02_posX = drone02.position.x
    d02_posY = drone02.position.y
    d02_posZ = drone02.position.z

    d03_vel = data.twist[4]
    d03_velLin_x = d03_vel.linear.x
    d03_velLin_y = d03_vel.linear.y

    d03_posX = drone03.position.x
    d03_posY = drone03.position.y
    d03_posZ = drone03.position.z

    d04_posX = drone04.position.x
    d04_posY = drone04.position.y
    d04_posZ = drone04.position.z

    d05_posX = drone05.position.x
    d05_posY = drone05.position.y
    d05_posZ = drone05.position.z

    d06_posX = drone06.position.x
    d06_posY = drone06.position.y
    d06_posZ = drone06.position.z

    d1_posX = drone1.position.x
    d1_posY = drone1.position.y
    d1_posZ = drone1.position.z
    
    d2_posX = drone2.position.x
    d2_posY = drone2.position.y
    d2_posZ = drone2.position.z

    d3_posX = drone3.position.x
    d3_posY = drone3.position.y
    d3_posZ = drone3.position.z

    if d1_posZ < 2:
        attract_x_d1 = 0.0
        attract_y_d1 = 0.0
        
    if d2_posZ < 2:
        attract_x_d2 = 0.0
        attract_y_d2 = 0.0

    if d3_posZ < 2:
        attract_x_d3 = 0.0
        attract_y_d3 = 0.0

    if d1_posZ >= 2:
        attract_x_d1, attract_y_d1 = calc_attractive_potential(d1_posX, d1_posY, d01_posX, d01_posY)

    if d2_posZ >= 2:
        attract_x_d2, attract_y_d2 = calc_attractive_potential(d2_posX, d2_posY, d02_posX, d02_posY)
    
    if d3_posZ >= 2:
        attract_x_d3, attract_y_d3 = calc_attractive_potential(d3_posX, d3_posY, d03_posX, d03_posY)
    

    
    repuls_x_d1, repuls_y_d1 = calc_repulsive_potential(d1_posX, d1_posY, d2_posX, d2_posY)
    repuls_x_d2, repuls_y_d2 = calc_repulsive_potential(d2_posX, d2_posY, d3_posX, d3_posY)
    repuls_x_d3, repuls_y_d3 = calc_repulsive_potential(d3_posX, d3_posY, d1_posX, d1_posY)
    new_velx_d2 = d02_velLin_x + attract_x_d2 + repuls_x_d2 - repuls_x_d1
    new_vely_d2 = d02_velLin_y + attract_y_d2 + repuls_y_d2 - repuls_y_d1
    new_velx_d1 = d01_velLin_x + attract_x_d1 + repuls_x_d1 - repuls_x_d3
    new_vely_d1 = d01_velLin_y + attract_y_d1 + repuls_y_d1 - repuls_y_d3
    new_velx_d3 = d03_velLin_x + attract_x_d3 + repuls_x_d3 - repuls_x_d2
    new_vely_d3 = d03_velLin_y + attract_y_d3 + repuls_y_d3 - repuls_y_d2

    
    calc_odom_d01 = (Vector3(-0.5*(d01_posX-0),-0.5*(d01_posY-10),-5*(d01_posZ-3)))
    calc_odom_d02 = (Vector3(-0.5*(d02_posX-8.6),-0.5*(d02_posY+5),-5*(d02_posZ-3)))
    calc_odom_d03 = (Vector3(-0.5*(d03_posX+8.6),-0.5*(d03_posY+5),-5*(d03_posZ-3)))
    calc_odom_d04 = (Vector3(-0.5*(d04_posX-8.6),-0.5*(d04_posY-5),-5*(d04_posZ-3)))
    calc_odom_d05 = (Vector3(-0.5*(d05_posX-0),-0.5*(d05_posY+10),-5*(d05_posZ-3)))
    calc_odom_d06 = (Vector3(-0.5*(d06_posX+8.6),-0.5*(d06_posY-5),-5*(d06_posZ-3)))
    calc_odom_d1 = (Vector3(new_velx_d1, new_vely_d1,-5*(d1_posZ-9)))
    calc_odom_d2 = (Vector3(new_velx_d2, new_vely_d2,-5*(d2_posZ-9)))
    calc_odom_d3 = (Vector3(new_velx_d3, new_vely_d3,-5*(d3_posZ-9)))
    

    pub01.publish(Twist(calc_odom_d01, Vector3(0,0,0)))
    pub02.publish(Twist(calc_odom_d02, Vector3(0,0,0)))
    pub03.publish(Twist(calc_odom_d03, Vector3(0,0,0)))
    pub04.publish(Twist(calc_odom_d04, Vector3(0,0,0)))
    pub05.publish(Twist(calc_odom_d05, Vector3(0,0,0)))
    pub06.publish(Twist(calc_odom_d06, Vector3(0,0,0)))
    pub1.publish(Twist(calc_odom_d1, Vector3(0,0,0)))
    pub2.publish(Twist(calc_odom_d2, Vector3(0,0,0)))
    pub3.publish(Twist(calc_odom_d3, Vector3(0,0,0)))
    
    
    
    rospy.Rate(10).sleep

def listener():
    rospy.init_node('listener', anonymous=False)
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback)
    rospy.spin()
    

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass 
