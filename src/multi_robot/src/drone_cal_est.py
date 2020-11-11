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
    return 0.5*(tx-x), 0.5*(ty-y)

def calc_repulsive_potential(x1, y1, x2, y2):
    dist = math.sqrt((x1-x2)**2 + (y1-y2)**2)
    if(dist<=2):
	force_x = 5*(dist)-10
	force_y = 5*(dist)-10
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
    #rospy.loginfo('%s', data.pose.pose.position.x)

    t_orienX = data.pose[2].orientation.x
    t_orienY = data.pose[2].orientation.y
    t_orienZ = data.pose[2].orientation.z
    t_orienW = data.pose[2].orientation.w
    (t_roll, t_pitch, t_yaw) = tf.transformations.euler_from_quaternion([t_orienX, t_orienY, t_orienZ,t_orienW])
    t_velLin = data.twist[2].linear
    t_velAngular = data.twist[2].angular
    t_posX = data.pose[2].position.x
    t_posY = data.pose[2].position.y
    t_posZ = data.pose[2].position.z
    t_odometryTuple = [t_posX,t_posY,t_posZ,t_velLin.x,t_velLin.y,t_velLin.z,t_velAngular.x,t_velAngular.y,t_velAngular.z,t_yaw]

    d2_orienX = data.pose[3].orientation.x
    d2_orienY = data.pose[3].orientation.y
    d2_orienZ = data.pose[3].orientation.z
    d2_orienW = data.pose[3].orientation.w
    (d2_roll, d2_pitch, d2_yaw) = tf.transformations.euler_from_quaternion([d2_orienX, d2_orienY, d2_orienZ,d2_orienW])
    d2_velLin = data.twist[3].linear
    d2_velAngular = data.twist[3].angular
    d2_posX = data.pose[3].position.x
    d2_posY = data.pose[3].position.y
    d2_posZ = data.pose[3].position.z
    d2_odometryTuple = [d2_posX,d2_posY,d2_posZ,d2_velLin.x,d2_velLin.y,d2_velLin.z,d2_velAngular.x,d2_velAngular.y,d2_velAngular.z,d2_yaw]

    d1_orienX = data.pose[4].orientation.x
    d1_orienY = data.pose[4].orientation.y
    d1_orienZ = data.pose[4].orientation.z
    d1_orienW = data.pose[4].orientation.w
    (d1_roll, d1_pitch, d1_yaw) = tf.transformations.euler_from_quaternion([d1_orienX, d1_orienY, d1_orienZ,d1_orienW])
    d1_velLin = data.twist[4].linear
    d1_velAngular = data.twist[4].angular
    d1_posX = data.pose[4].position.x
    d1_posY = data.pose[4].position.y
    d1_posZ = data.pose[4].position.z
    d1_odometryTuple = [d1_posX,d1_posY,d1_posZ,d1_velLin.x,d1_velLin.y,d1_velLin.z,d1_velAngular.x,d1_velAngular.y,d1_velAngular.z,d1_yaw]

    attract_x_d1, attract_y_d1 = calc_attractive_potential(d1_posX, d1_posY, t_posX, t_posY)
    attract_x_d2, attract_y_d2 = calc_attractive_potential(d2_posX, d2_posY, t_posX, t_posY)
    repuls_x_d1, repuls_y_d1 = calc_repulsive_potential(d1_posX+attract_x_d1*0.5, d1_posY+attract_y_d1*0.5, d2_posX+attract_x_d2*0.5, d2_posY+attract_y_d2*0.5)
    repuls_x_d2, repuls_y_d2 = calc_repulsive_potential(d2_posX+attract_x_d2*0.5, d2_posY+attract_y_d2*0.5, d1_posX+attract_x_d1*0.5, d1_posY+attract_y_d1*0.5)
    new_velx_d2 = attract_x_d2 + repuls_x_d2
    new_vely_d2 = attract_y_d2 + repuls_y_d2
    new_velx_d1 = attract_x_d1 + repuls_x_d1
    new_vely_d1 = attract_y_d1 + repuls_y_d1
    
    if d2_posZ>5:
        calc_odom_d2 = (Vector3(new_velx_d2, new_vely_d2,-0.5))
    if d2_posZ<=5: 
        calc_odom_d2 = (Vector3(new_velx_d2, new_vely_d2,0.05))
    if d1_posZ>5:
        calc_odom_d1 = (Vector3(new_velx_d1, new_vely_d1,-0.5))
    if d1_posZ<=5: 
        calc_odom_d1 = (Vector3(new_velx_d1, new_vely_d1,0.05))
    pub1.publish(Twist(calc_odom_d1, Vector3(0,0,0)))
    rospy.Rate(1).sleep
    pub2.publish(Twist(calc_odom_d2, Vector3(0,0,0)))
    rospy.Rate(1).sleep

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
