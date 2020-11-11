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
   
    
    tb3_pose = data.pose[1]
    tb3_vel = data.twist[1]
    drone1 = data.pose[2]
  

    t_velLin_x = tb3_vel.linear.x
    t_velLin_y = tb3_vel.linear.y
    t_posX = tb3_pose.position.x
    t_posY = tb3_pose.position.y
    t_posZ = tb3_pose.position.z

    d1_posX = drone1.position.x
    d1_posY = drone1.position.y
    d1_posZ = drone1.position.z
    
  
    attract_x_d1, attract_y_d1 = calc_attractive_potential(d1_posX, d1_posY, t_posX, t_posY)
    repuls_x_d1, repuls_y_d1 = calc_repulsive_potential(d1_posX, d1_posY, d2_posX, d2_posY)
    new_velx_d1 = t_velLin_x + attract_x_d1 + repuls_x_d1 - repuls_x_d3
    new_vely_d1 = t_velLin_y + attract_y_d1 + repuls_y_d1 - repuls_y_d3

    if d1_posZ>7:
        calc_odom_d1 = (Vector3(new_velx_d1, new_vely_d1,-0.5))
    if d1_posZ<7: 
        calc_odom_d1 = (Vector3(new_velx_d1, new_vely_d1,0.05))

    pub1.publish(Twist(calc_odom_d1, Vector3(0,0,0)))

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
