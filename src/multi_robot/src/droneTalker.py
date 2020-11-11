#!/usr/bin/env python

import time
import sys
import csv

# ROS Interface

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

fwdVel = 0.0
angVel = 0.0
pubRate = 30


def talker():

       pub = rospy.Publisher('drone/cmd_vel',Twist,queue_size=10)
       rospy.init_node('act', anonymous=True)
       rate = rospy.Rate(pubRate) #hz
       while not rospy.is_shutdown():
           with open("/home/robolab/catkin_ws/src/multi_robot/src/test.csv", 'r') as f:
              try:
                  reader = csv.reader(f)
                  velList = list(reader)
                  print "New cmnd", velList
                  linX = float(velList[0][0])
                  linY = float(velList[0][1])
                  linZ = float(velList[0][2])
                  angX = float(velList[0][3])
                  angY = float(velList[0][4])
                  angZ = float(velList[0][5])
                  pub.publish(Twist(Vector3(linX,linY,linZ),Vector3(angX,angY,angZ)))
              except: 
                   pass 
           rate.sleep()
           #time.sleep(0.1)


if __name__ == '__main__':
    try:
       talker()
    except rospy.ROSInterruptException:
       pass 

