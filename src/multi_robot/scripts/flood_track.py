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
    pub07 = rospy.Publisher('drone07/cmd_vel',Twist,queue_size=10)
    pub08 = rospy.Publisher('drone08/cmd_vel',Twist,queue_size=10)
        
    drone01 = data.pose[19]
    drone02 = data.pose[20]
    drone03 = data.pose[21]
    drone04 = data.pose[22]
    drone05 = data.pose[23]
    drone06 = data.pose[24]
    drone07 = data.pose[25]
    drone08 = data.pose[26]
    
    
    d01_posX = drone01.position.x
    d01_posY = drone01.position.y
    d01_posZ = drone01.position.z

    d02_posX = drone02.position.x
    d02_posY = drone02.position.y
    d02_posZ = drone02.position.z

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

    d07_posX = drone07.position.x
    d07_posY = drone07.position.y
    d07_posZ = drone07.position.z

    d08_posX = drone08.position.x
    d08_posY = drone08.position.y
    d08_posZ = drone08.position.z
    
    calc_odom_d01 = (Vector3(-0.5*(d01_posX),-0.5*(d01_posY-10),-5*(d01_posZ-5)))
    calc_odom_d02 = (Vector3(-0.5*(d02_posX-10),-0.5*(d02_posY),-5*(d02_posZ-5)))
    calc_odom_d03 = (Vector3(-0.5*(d03_posX),-0.5*(d03_posY+10),-5*(d03_posZ-5)))
    calc_odom_d04 = (Vector3(-0.5*(d04_posX+10),-0.5*(d04_posY),-5*(d04_posZ-5)))
    calc_odom_d05 = (Vector3(-0.5*(d05_posX-10),-0.5*(d05_posY-10),-5*(d05_posZ-5)))
    calc_odom_d06 = (Vector3(-0.5*(d06_posX-10),-0.5*(d06_posY+10),-5*(d06_posZ-5)))
    calc_odom_d07 = (Vector3(-0.5*(d07_posX+10),-0.5*(d07_posY+10),-5*(d07_posZ-5)))
    calc_odom_d08 = (Vector3(-0.5*(d08_posX+10),-0.5*(d08_posY-10),-5*(d08_posZ-5)))
    

    pub01.publish(Twist(calc_odom_d01, Vector3(0,0,0)))
    pub02.publish(Twist(calc_odom_d02, Vector3(0,0,0)))
    pub03.publish(Twist(calc_odom_d03, Vector3(0,0,0)))
    pub04.publish(Twist(calc_odom_d04, Vector3(0,0,0)))
    pub05.publish(Twist(calc_odom_d05, Vector3(0,0,0)))
    pub06.publish(Twist(calc_odom_d06, Vector3(0,0,0)))
    pub07.publish(Twist(calc_odom_d07, Vector3(0,0,0)))
    pub08.publish(Twist(calc_odom_d08, Vector3(0,0,0)))
    
    
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
