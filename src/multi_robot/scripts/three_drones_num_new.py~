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
    pub1 = rospy.Publisher('drone1/cmd_vel',Twist,queue_size=10)
    pub2 = rospy.Publisher('drone2/cmd_vel',Twist,queue_size=10)
    pub3 = rospy.Publisher('drone3/cmd_vel',Twist,queue_size=10)
    
    tb3_pose = data.pose[4]
    tb3_vel = data.twist[4]
    drone1 = data.pose[1]
    drone2 = data.pose[2]
    drone3 = data.pose[3]

    t_velLin_x = tb3_vel.linear.x
    t_velLin_y = tb3_vel.linear.y
    t_posX = tb3_pose.position.x
    t_posY = tb3_pose.position.y
    t_posZ = tb3_pose.position.z

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
        attract_x_d1, attract_y_d1 = calc_attractive_potential(d1_posX, d1_posY, t_posX, t_posY)

    if d2_posZ >= 2:
        attract_x_d2, attract_y_d2 = calc_attractive_potential(d2_posX, d2_posY, t_posX, t_posY)
    
    if d3_posZ >= 2:
        attract_x_d3, attract_y_d3 = calc_attractive_potential(d3_posX, d3_posY, t_posX, t_posY)
    

    
    repuls_x_d1, repuls_y_d1 = calc_repulsive_potential(d1_posX, d1_posY, d2_posX, d2_posY)
    repuls_x_d2, repuls_y_d2 = calc_repulsive_potential(d2_posX, d2_posY, d3_posX, d3_posY)
    repuls_x_d3, repuls_y_d3 = calc_repulsive_potential(d3_posX, d3_posY, d1_posX, d1_posY)
    new_velx_d2 = t_velLin_x + attract_x_d2 + repuls_x_d2 - repuls_x_d1
    new_vely_d2 = t_velLin_y + attract_y_d2 + repuls_y_d2 - repuls_y_d1
    new_velx_d1 = t_velLin_x + attract_x_d1 + repuls_x_d1 - repuls_x_d3
    new_vely_d1 = t_velLin_y + attract_y_d1 + repuls_y_d1 - repuls_y_d3
    new_velx_d3 = t_velLin_x + attract_x_d3 + repuls_x_d3 - repuls_x_d2
    new_vely_d3 = t_velLin_y + attract_y_d3 + repuls_y_d3 - repuls_y_d2

    
    calc_odom_d1 = (Vector3(new_velx_d1, new_vely_d1,-5*(d1_posZ-7)))
    calc_odom_d2 = (Vector3(new_velx_d2, new_vely_d2,-5*(d2_posZ-9)))
    calc_odom_d3 = (Vector3(new_velx_d3, new_vely_d3,-5*(d3_posZ-11)))
    
    
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
