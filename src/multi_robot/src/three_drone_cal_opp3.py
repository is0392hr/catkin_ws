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
    return dist*(tx-x), dist*(ty-y)

def calc_repulsive_potential(x1, y1, x2, y2):
    dist = math.sqrt((x1-x2)**2 + (y1-y2)**2)
    if(dist<=4):
	force_x = 5*(math.sqrt((x1-x2)**2))-20
	force_y = 5*(math.sqrt((y1-y2)**2))-20
        if(x2>x1):
	    force_x = -force_x
        if(y2>y1):
	    force_y = -force_y
    else:
	force_x=0.0 
	force_y=0.0

    return force_x, force_y

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + '%s', data.twist[3])
    pub1 = rospy.Publisher('drone1/cmd_vel',Twist,queue_size=10)
    pub2 = rospy.Publisher('drone2/cmd_vel',Twist,queue_size=10)
    pub3 = rospy.Publisher('drone3/cmd_vel',Twist,queue_size=10)
    #rospy.loginfo('%s', data.pose.pose.position.x)

    t_orienX = data.pose[4].orientation.x
    t_orienY = data.pose[4].orientation.y
    t_orienZ = data.pose[4].orientation.z
    t_orienW = data.pose[4].orientation.w
    t_velLin_x = data.twist[4].linear.x
    t_velLin_y = data.twist[4].linear.y
    t_velAngular = data.twist[4].angular
    t_posX = data.pose[4].position.x
    t_posY = data.pose[4].position.y
    t_posZ = data.pose[4].position.z
    
    d2_orienX = data.pose[2].orientation.x
    d2_orienY = data.pose[2].orientation.y
    d2_orienZ = data.pose[2].orientation.z
    d2_orienW = data.pose[2].orientation.w
    d2_velLin = data.twist[2].linear
    d2_velAngular = data.twist[2].angular
    d2_posX = data.pose[2].position.x
    d2_posY = data.pose[2].position.y
    d2_posZ = data.pose[2].position.z

    d1_orienX = data.pose[5].orientation.x
    d1_orienY = data.pose[5].orientation.y
    d1_orienZ = data.pose[5].orientation.z
    d1_orienW = data.pose[5].orientation.w
    d1_velLin = data.twist[5].linear
    d1_velAngular = data.twist[5].angular
    d1_posX = data.pose[5].position.x
    d1_posY = data.pose[5].position.y
    d1_posZ = data.pose[5].position.z

    d3_orienX = data.pose[3].orientation.x
    d3_orienY = data.pose[3].orientation.y
    d3_orienZ = data.pose[3].orientation.z
    d3_orienW = data.pose[3].orientation.w
    d3_velLin = data.twist[3].linear
    d3_velAngular = data.twist[3].angular
    d3_posX = data.pose[3].position.x
    d3_posY = data.pose[3].position.y
    d3_posZ = data.pose[3].position.z

    attract_x_d1, attract_y_d1 = calc_attractive_potential(d1_posX, d1_posY, t_posX, t_posY)
    attract_x_d2, attract_y_d2 = calc_attractive_potential(d2_posX, d2_posY, t_posX, t_posY)
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

    if d2_posZ>5:
        calc_odom_d2 = (Vector3(new_velx_d2, new_vely_d2,-0.5))
    if d2_posZ<=5: 
        calc_odom_d2 = (Vector3(new_velx_d2, new_vely_d2,0.05))
    if d1_posZ>5:
        calc_odom_d1 = (Vector3(new_velx_d1, new_vely_d1,-0.5))
    if d1_posZ<=5: 
        calc_odom_d1 = (Vector3(new_velx_d1, new_vely_d1,0.05))
    if d3_posZ>5:
        calc_odom_d3 = (Vector3(new_velx_d3, new_vely_d3,-0.5))
    if d3_posZ<=5: 
        calc_odom_d3 = (Vector3(new_velx_d3, new_vely_d3,0.05))
    pub1.publish(Twist(calc_odom_d1, Vector3(0,0,0)))
    pub2.publish(Twist(calc_odom_d2, Vector3(0,0,0)))
    pub3.publish(Twist(calc_odom_d3, Vector3(0,0,0)))
    rospy.Rate(10).sleep

def listener():
    rospy.init_node('listener', anonymous=False)
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback)
    # Use the following piece to get the position from Odometry. Note
    #rospy.Subscriber('/odom', Odometry, callbackOdometry)
    rospy.spin()
    

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass 
