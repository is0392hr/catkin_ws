#!/usr/bin/env python3

import rospy
import os 
import math
from std_msgs.msg import Float64MultiArray, String
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import numpy as np

import cv2, cv_bridge
from sensor_msgs.msg import Image, CameraInfo, CompressedImage

from segmentation import predict

import configparser
import time


control = [0,0,0]
targets, centroid, key = [], [], []

# model = predict.model_from_checkpoint_path("/home/koki/catkin_ws/src/multi_robot_unity/scripts/checkpoints/")

def get_params():
    config = configparser.ConfigParser()
    config.read(os.path.join(os.path.dirname(__file__), '/home/koki/catkin_ws/src/multi_robot_unity/scripts/index.txt'))
    item_list = config.items('index')

    for item in item_list:
        key = item[0]
        value = item[1]

        if key == 'drone01':
            d1 = int(value)
        if key == 'drone02':
            d2 = int(value)    
        if key == 'drone03':
            d3 = int(value)
        if key == 'drone04':
            d4 = int(value)
        if key == 'drone05':
            d5 = int(value)
        if key == 'drone06':
            d6 = int(value)
        if key == 'drone07':
            d7 = int(value)
        if key == 'drone08':
            d8 = int(value)
        if key == 'drone09':
            d9 = int(value)
        if key == 'drone10':
            d10 = int(value)
        if key == 'drone11':
            d11 = int(value)
        if key == 'drone12':
            d12 = int(value)
        if key == 'drone13':
            d13 = int(value)
        if key == 'drone14':
            d14 = int(value)
        if key == 'drone15':
            d15 = int(value)
        if key == 'drone16':
            d16 = int(value)
        if key == 'drone17':
            d17 = int(value)


    params = [d1,d2,d3,d4,d5,d6,d7,d8,d9,d10,d11,d12,d13,d14,d15,d16,d17]
    return params


idx = get_params()

print(idx)

class Follower:
    def __init__(self):
    
        self.bridge = cv_bridge.CvBridge()
        # image = rospy.Subscriber('/unity_image/compressed_d2', CompressedImage, self.image_callback)
        self.pos = rospy.Subscriber("controler", Float64MultiArray, self.pos_callback)
        # self.centroid = rospy.Subscriber("centroid", Float64MultiArray, self.centroid_callback)
        # self.key = rospy.Subscriber("key", String,self.key_callback)
        self.data = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.cmd_vel_pub = rospy.Publisher('drone02/cmd_vel',Twist, queue_size=10)
        self.twist = Twist()
     
    def pos_callback(self, data):
        print(data.data)
        global targets
        targets = np.array(data.data)
        targets = targets.reshape([-1,2])   
       
    # def centroid_callback(self, data):
    #     #print(data.data)
    #     global centroid
    #     centroid = np.array(data.data)
        
              
    # def key_callback(self, data):
    #     #print(data.data)
    #     global key
    #     key = data.data

    # def image_callback(self, msg):

    #     image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8') #bgr8

    #     global model
    #     seg_img = predict.action(model, image)
    #     seg_img = cv2.resize(1920, 1080)
    #     cv2.imshow("d2", image)
    #     cv2.waitKey(3)
    

    def callback(self, data):

        global key
        drone02 = data.pose[idx[1]]
        drone02_vel = data.twist[idx[1]]
        d02_posX = drone02.position.x
        d02_posY = drone02.position.y
        d02_posZ = drone02.position.z
        d02_velX = drone02_vel.linear.x
        d02_velY = drone02_vel.linear.y

        global targets
        target = targets[1]
        posX = target[0]
        posY = target[1]
        
        errX = posX - d02_posX
        errY = posY - d02_posY

        print("target: ", target, "Err: ", (errX, errY))

        ####################    
        ## POSITION BASED ##
        ####################
        

        calc_odom = (Vector3(-0.3*(d02_posX-posX), -0.3*(d02_posY-posY),-3*(d02_posZ-8)))
        pub = self.cmd_vel_pub
        pub.publish(Twist(calc_odom, Vector3(0,0,0)))
        #print('position control now')
        rospy.Rate(10).sleep
           

    
def listener():
    rospy.init_node('d_2')
    follower = Follower()
    rospy.spin()

if __name__ == '__main__':
    start = time.time()
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
# END ALL
