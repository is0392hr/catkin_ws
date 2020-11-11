#!/usr/bin/env python

import os
import sys
import tempfile
import csv
import tf
import rospy

from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

filename = "/home/robolab/catkin_ws/src/multi_robot/src/dataExchangeROS2Math.csv"

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + '%s', data)
    #rospy.loginfo('%s', data.pose.pose.position.x)
    with tempfile.NamedTemporaryFile('w', dir=os.path.dirname(filename),delete=False) as fake_file:
        with fake_file as fake_csv:
            writer = csv.writer(fake_csv, delimiter=',')
            orienX = data.pose[3].orientation.x
            orienY = data.pose[3].orientation.y
            orienZ = data.pose[3].orientation.z
            orienW = data.pose[3].orientation.w
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orienX, orienY, orienZ,orienW])
            velLin = data.twist[3].linear
            velAngular = data.twist[3].angular
            posX = data.pose[3].position.x
            posY = data.pose[3].position.y
            posZ = data.pose[3].position.z
            odometryTuple = [posX,posY,posZ,velLin.x,velLin.y,velLin.z,velAngular.x,velAngular.y,velAngular.z,yaw]
            writer.writerow(odometryTuple)
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
