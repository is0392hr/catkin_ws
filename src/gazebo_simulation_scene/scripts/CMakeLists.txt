#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
rospy.init_node('odometry_publisher')
odom_pub = rospy.Publisher("odom", Odometry, queue_size=60)
rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
odom = Odometry()
#header = Header()
odom.header.frame_id = "odom"
odom.child_frame_id = "base_link"
model = GetModelStateRequest()
model.model_name = 'quadrotor0'
r = rospy.Rate(60)