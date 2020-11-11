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

filename = "/home/robolab/catkin_ws/src/multi_robot/src/test.csv"

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + '%s', data)
    #rospy.loginfo('%s', data.pose.pose.position.x)
    with tempfile.NamedTemporaryFile('w', dir=os.path.dirname(filename),delete=False) as fake_file:
        with fake_file as fake_csv:
            writer = csv.writer(fake_csv, delimiter=',')
            d_orienX = data.pose[3].orientation.x
            d_orienY = data.pose[3].orientation.y
            d_orienZ = data.pose[3].orientation.z
            d_orienW = data.pose[3].orientation.w
            (d_roll, d_pitch, d_yaw) = tf.transformations.euler_from_quaternion([d_orienX, d_orienY, d_orienZ,d_orienW])
            d_velLin = data.twist[3].linear
            d_velAngular = data.twist[3].angular
            d_posX = data.pose[3].position.x
            d_posY = data.pose[3].position.y
            d_posZ = data.pose[3].position.z
            d_odometryTuple = [d_posX,d_posY,d_posZ,d_velLin.x,d_velLin.y,d_velLin.z,d_velAngular.x,d_velAngular.y,d_velAngular.z,d_yaw]
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
	    distance = math.sqrt((t_posX-d_posX)**2) + math.sqrt((t_posY-d_posY)**2)
	    theta = t_yaw-d_yaw
	    sin = math.sin(theta)
	    cos = math.cos(theta)
	    new_velx = t_velLin.x + (t_posX-d_posX)
	    new_vely = t_velLin.y + (t_posY-d_posY)
            calc_odom = [new_velx, new_vely,0.001,0,0,0]
            writer.writerow(calc_odom)
        tempFileName = fake_file.name
    os.rename(tempFileName, filename)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback)
    # Use the following piece to get the position from Odometry. Note
    #rospy.Subscriber('/odom', Odometry, callbackOdometry)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass 
