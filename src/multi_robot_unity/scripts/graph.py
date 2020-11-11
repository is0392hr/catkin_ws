#!/usr/bin/env python

import os
import sys
import tempfile
import csv
import pandas as pd
import tf
import rospy
import math
import message_filters
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import Twist, Vector3
import numpy as np

import cv2, cv_bridge
from sensor_msgs.msg import Image, CameraInfo

import configparser
import time

#import networks as nx
import matplotlib.pyplot as plt


MAX_DIST = 20


control = [0,0,0]
global control
global count
count = 0

def get_params():
    config = configparser.ConfigParser()
    config.read('index.txt')
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

    params = [d1,d2,d3,d4,d5,d6,d7,d8, d9]
    return params


idx = get_params()

print(idx)

class Graph:
    def __init__(self):

        self.data = rospy.Subscriber('/gazebo/model_states', ModelStates, callback=self.plot)
        
    def plot(self, data):
        drone01 = data.pose[idx[0]]
        drone01_vel = data.twist[idx[0]]
        d01_posX = drone01.position.x
        d01_posY = drone01.position.y
        d01_posZ = drone01.position.z
        d01_velX = drone01_vel.linear.x
        d01_velY = drone01_vel.linear.y
        
        drone02 = data.pose[idx[1]]
        drone02_vel = data.twist[idx[1]]
        d02_posX = drone02.position.x
        d02_posY = drone02.position.y
        d02_posZ = drone02.position.z
        d02_velX = drone02_vel.linear.x
        d02_velY = drone02_vel.linear.y

        drone04 = data.pose[idx[3]]
        drone04_vel = data.twist[idx[3]]
        d04_posX = drone04.position.x
        d04_posY = drone04.position.y
        d04_posZ = drone04.position.z
        d04_velX = drone04_vel.linear.x
        d04_velY = drone04_vel.linear.y

        drone05 = data.pose[idx[4]]
        drone05_vel = data.twist[idx[4]]
        d05_posX = drone05.position.x
        d05_posY = drone05.position.y
        d05_posZ = drone05.position.z
        d05_velX = drone05_vel.linear.x
        d05_velY = drone05_vel.linear.y

        drone06 = data.pose[idx[5]]
        drone06_vel = data.twist[idx[5]]
        d06_posX = drone06.position.x
        d06_posY = drone06.position.y
        d06_posZ = drone06.position.z
        d06_velX = drone06_vel.linear.x
        d06_velY = drone06_vel.linear.y

        drone07 = data.pose[idx[6]]
        drone07_vel = data.twist[idx[6]]
        d07_posX = drone07.position.x
        d07_posY = drone07.position.y
        d07_posZ = drone07.position.z
        d07_velX = drone07_vel.linear.x
        d07_velY = drone07_vel.linear.y

        drone08 = data.pose[idx[7]]
        drone08_vel = data.twist[idx[7]]
        d08_posX = drone08.position.x
        d08_posY = drone08.position.y
        d08_posZ = drone08.position.z
        d08_velX = drone08_vel.linear.x
        d08_velY = drone08_vel.linear.y

        drone09 = data.pose[idx[8]]
        drone09_vel = data.twist[idx[8]]
        d09_posX = drone09.position.x
        d09_posY = drone09.position.y
        d09_posZ = drone09.position.z
        d09_velX = drone09_vel.linear.x
        d09_velY = drone09_vel.linear.y

        d_12 = float(math.sqrt((d02_posX-d01_posX)**2 + (d02_posY-d01_posY)**2))
        d_14 = float(math.sqrt((d04_posX-d01_posX)**2 + (d04_posY-d01_posY)**2))
        d_15 = float(math.sqrt((d05_posX-d01_posX)**2 + (d05_posY-d01_posY)**2))
        d_16 = float(math.sqrt((d06_posX-d01_posX)**2 + (d06_posY-d01_posY)**2))
        d_17 = float(math.sqrt((d07_posX-d01_posX)**2 + (d07_posY-d01_posY)**2))
        d_18 = float(math.sqrt((d08_posX-d01_posX)**2 + (d08_posY-d01_posY)**2))
        d_19 = float(math.sqrt((d09_posX-d01_posX)**2 + (d09_posY-d01_posY)**2))

        d_21 = float(math.sqrt((d01_posX-d02_posX)**2 + (d01_posY-d02_posY)**2))
        d_24 = float(math.sqrt((d04_posX-d02_posX)**2 + (d04_posY-d02_posY)**2))
        d_25 = float(math.sqrt((d05_posX-d02_posX)**2 + (d05_posY-d02_posY)**2))
        d_26 = float(math.sqrt((d06_posX-d02_posX)**2 + (d06_posY-d02_posY)**2))
        d_27 = float(math.sqrt((d07_posX-d02_posX)**2 + (d07_posY-d02_posY)**2))
        d_28 = float(math.sqrt((d08_posX-d02_posX)**2 + (d08_posY-d02_posY)**2))
        d_29 = float(math.sqrt((d09_posX-d02_posX)**2 + (d09_posY-d02_posY)**2))

        d_41 = float(math.sqrt((d01_posX-d04_posX)**2 + (d01_posY-d04_posY)**2))
        d_42 = float(math.sqrt((d02_posX-d04_posX)**2 + (d02_posY-d04_posY)**2))
        d_45 = float(math.sqrt((d05_posX-d04_posX)**2 + (d05_posY-d04_posY)**2))
        d_46 = float(math.sqrt((d06_posX-d04_posX)**2 + (d06_posY-d04_posY)**2))
        d_47 = float(math.sqrt((d07_posX-d04_posX)**2 + (d07_posY-d04_posY)**2))
        d_48 = float(math.sqrt((d08_posX-d04_posX)**2 + (d08_posY-d04_posY)**2))
        d_49 = float(math.sqrt((d09_posX-d04_posX)**2 + (d09_posY-d04_posY)**2))
        
        d_51 = float(math.sqrt((d01_posX-d05_posX)**2 + (d01_posY-d05_posY)**2))
        d_52 = float(math.sqrt((d02_posX-d05_posX)**2 + (d02_posY-d05_posY)**2))
        d_54 = float(math.sqrt((d04_posX-d05_posX)**2 + (d04_posY-d05_posY)**2))
        d_56 = float(math.sqrt((d06_posX-d05_posX)**2 + (d06_posY-d05_posY)**2))
        d_57 = float(math.sqrt((d07_posX-d05_posX)**2 + (d07_posY-d05_posY)**2))
        d_58 = float(math.sqrt((d08_posX-d05_posX)**2 + (d08_posY-d05_posY)**2))
        d_59 = float(math.sqrt((d09_posX-d05_posX)**2 + (d09_posY-d05_posY)**2))


        d_61 = float(math.sqrt((d01_posX-d06_posX)**2 + (d01_posY-d06_posY)**2))
        d_62 = float(math.sqrt((d02_posX-d06_posX)**2 + (d02_posY-d06_posY)**2))
        d_64 = float(math.sqrt((d04_posX-d06_posX)**2 + (d04_posY-d06_posY)**2))
        d_65 = float(math.sqrt((d05_posX-d06_posX)**2 + (d05_posY-d06_posY)**2))
        d_67 = float(math.sqrt((d07_posX-d06_posX)**2 + (d07_posY-d06_posY)**2))
        d_68 = float(math.sqrt((d08_posX-d06_posX)**2 + (d08_posY-d06_posY)**2))
        d_69 = float(math.sqrt((d09_posX-d06_posX)**2 + (d09_posY-d06_posY)**2))

 
        d_71 = float(math.sqrt((d01_posX-d07_posX)**2 + (d01_posY-d07_posY)**2))
        d_72 = float(math.sqrt((d02_posX-d07_posX)**2 + (d02_posY-d07_posY)**2))
        d_74 = float(math.sqrt((d04_posX-d07_posX)**2 + (d04_posY-d07_posY)**2))
        d_75 = float(math.sqrt((d05_posX-d07_posX)**2 + (d05_posY-d07_posY)**2))
        d_76 = float(math.sqrt((d06_posX-d07_posX)**2 + (d06_posY-d07_posY)**2))
        d_78 = float(math.sqrt((d08_posX-d07_posX)**2 + (d08_posY-d07_posY)**2))
        d_79 = float(math.sqrt((d09_posX-d07_posX)**2 + (d09_posY-d07_posY)**2))


        d_81 = float(math.sqrt((d01_posX-d08_posX)**2 + (d01_posY-d08_posY)**2))
        d_82 = float(math.sqrt((d02_posX-d08_posX)**2 + (d02_posY-d08_posY)**2))
        d_84 = float(math.sqrt((d04_posX-d08_posX)**2 + (d04_posY-d08_posY)**2))
        d_85 = float(math.sqrt((d05_posX-d08_posX)**2 + (d05_posY-d08_posY)**2))
        d_86 = float(math.sqrt((d06_posX-d08_posX)**2 + (d06_posY-d08_posY)**2))
        d_87 = float(math.sqrt((d07_posX-d08_posX)**2 + (d07_posY-d08_posY)**2))
        d_89 = float(math.sqrt((d09_posX-d08_posX)**2 + (d09_posY-d08_posY)**2))


        d_91 = float(math.sqrt((d01_posX-d09_posX)**2 + (d01_posY-d09_posY)**2))
        d_92 = float(math.sqrt((d02_posX-d09_posX)**2 + (d02_posY-d09_posY)**2))
        d_94 = float(math.sqrt((d04_posX-d09_posX)**2 + (d04_posY-d09_posY)**2))
        d_95 = float(math.sqrt((d05_posX-d09_posX)**2 + (d05_posY-d09_posY)**2))
        d_96 = float(math.sqrt((d06_posX-d09_posX)**2 + (d06_posY-d09_posY)**2))
        d_97 = float(math.sqrt((d07_posX-d09_posX)**2 + (d07_posY-d09_posY)**2))
        d_98 = float(math.sqrt((d08_posX-d09_posX)**2 + (d08_posY-d09_posY)**2))


        D1 = [d_12, d_14, d_15, d_16, d_17, d_18, d_19, 
            d_24, d_25, d_26, d_27, d_28, d_29, 
            d_45, d_46, d_47, d_48, d_49,
            d_56, d_57, d_58, d_59, 
            d_67, d_68, d_69,
            d_78, d_79,
            d_89
        ] # [d_12, d_14, d_15, d_16, d_17, d_18, d_19]
        d1_min = min(D1)
        t = time.time() - start
        print(t, d1_min)
        
        #ax1 = fig.add_subplot(1,1,1)
        # ax1.plot(t, d1_min, linestyle='solid', linewidth = 1.0, marker='o', color='red')
        # ax1.xlabel('time [s]')
        # ax1.ylabel('Minimum distance of all drone distances [m]')
        # ax1.ylim(-5, 20)
        # ax1.pause(0.01)

        if t < 200:
            D1 = [t, d_12, d_14, d_15, d_16, d_17, d_18, d_19]
            df1 = pd.DataFrame(D1)
            df1.to_csv('out_d1.csv', mode='a', header=False)

            D2 = [t, d_21, d_24, d_25, d_26, d_27, d_28, d_29]
            df2 = pd.DataFrame(D2)
            df2.to_csv('out_d2.csv', mode='a', header=False)

            D4 = [t, d_41, d_42, d_45, d_46, d_47, d_48, d_49]
            df4 = pd.DataFrame(D4)
            df4.to_csv('out_d4.csv', mode='a', header=False)

            D5 = [t, d_51, d_52, d_54, d_56, d_57, d_58, d_59]
            df5 = pd.DataFrame(D5)
            df5.to_csv('out_d5.csv', mode='a', header=False)

            D6 = [t, d_61, d_62, d_64, d_65, d_67, d_68, d_69]
            df6 = pd.DataFrame(D6)
            df6.to_csv('out_d6.csv', mode='a', header=False)

            D7 = [t, d_71, d_72, d_74, d_75, d_76, d_78, d_79]
            df7 = pd.DataFrame(D7)
            df7.to_csv('out_d7.csv', mode='a', header=False)

            D8 = [t, d_81, d_82, d_84, d_85, d_86, d_87, d_89]
            df8 = pd.DataFrame(D8)
            df8.to_csv('out_d8.csv', mode='a', header=False)

            D9 = [t, d_91, d_92, d_94, d_95, d_96, d_97, d_98]
            df9 = pd.DataFrame(D9)
            df9.to_csv('out_d9.csv', mode='a', header=False)


        # m = np.array([[0, d_12, d_14, d_15, d_16, d_17, d_18, d_19],
        #     [d_21, 0, d_24, d_25, d_26, d_27, d_28, d_29],
        #     [d_41, d_42, 0, d_45, d_46, d_47, d_48, d_49],
        #     [d_51, d_52, d_54, 0, d_56, d_57, d_58, d_59],
        #     [d_61, d_62, d_64, d_65, 0, d_67, d_68, d_69],
        #     [d_71, d_72, d_74, d_75, d_76, 0, d_78, d_79],
        #     [d_81, d_82, d_84, d_85, d_86, d_87, 0, d_89],
        #     [d_91, d_92, d_94, d_95, d_96, d_97, d_98, 0]
        #     ])
        # nodes = np.array(['d1', 'd2', 'd4', 'd5', 'd6', 'd7', 'd8', 'd9'])
        # G = nx.Graph()
        # G.add_nodes_from(nodes)
        # edges = []

        # for hi, hv  in enumerate(m):
        #     for wi, wv in enumerate(hv):
        #         if(wv): edges.append((nodes[hi], nodes[wi]))
        # pos = nx.spring_layout(G)
        # nx.draw_networkx(G, pos, with_labels=True)
        # ax2 = fig.add_subplot(1,2,1)
        # ax2.axis("off")
        # ax2.pause(0.01)

        #if t< 0.5:
        '''
        fig= plt.figure()
        plt.ion()
        ax1 = fig.add_subplot(2,4,1)
        ax2 = fig.add_subplot(2,4,2)
        '''
        '''
        plt.plot(t, d_12, marker='o', color="red", label='d_12')
        plt.plot(t, d_14, marker='o', color="orange", label='d_14')
        plt.plot(t, d_15, marker='o', color="yellow", label='d_15')
        plt.plot(t, d_16, marker='o', color="green", label='d_16')
        plt.plot(t, d_17, marker='o', color="blue", label='d_17')
        plt.plot(t, d_18, marker='o', color="purple", label='d_18')
        plt.plot(t, d_19, marker='o', color="brown", label='d_19')
        
        plt.plot(t, d_21, marker='o', color="red", label='d_21')
        plt.plot(t, d_24, marker='o', color="orange", label='d_24')
        plt.plot(t, d_25, marker='o', color="yellow", label='d_25')
        plt.plot(t, d_26, marker='o', color="green", label='d_26')
        plt.plot(t, d_27, marker='o', color="blue", label='d_27')
        plt.plot(t, d_28, marker='o', color="purple", label='d_28')
        plt.plot(t, d_29, marker='o', color="brown", label='d_29') 
        
        plt.plot(t, d_14, marker='o', color="red", label='d_41')
        plt.plot(t, d_24, marker='o', color="orange", label='d_42')
        plt.plot(t, d_54, marker='o', color="yellow", label='d_45')
        plt.plot(t, d_64, marker='o', color="green", label='d_46')
        plt.plot(t, d_74, marker='o', color="blue", label='d_47')
        plt.plot(t, d_84, marker='o', color="purple", label='d_48')
        plt.plot(t, d_94, marker='o', color="brown", label='d_49')
        
        plt.plot(t, d_15, marker='o', color="red", label='d_51')
        plt.plot(t, d_25, marker='o', color="orange", label='d_52')
        plt.plot(t, d_45, marker='o', color="yellow", label='d_54')
        plt.plot(t, d_65, marker='o', color="green", label='d_56')
        plt.plot(t, d_75, marker='o', color="blue", label='d_57')
        plt.plot(t, d_85, marker='o', color="purple", label='d_58')
        plt.plot(t, d_95, marker='o', color="brown", label='d_59')
        
        plt.plot(t, d_16, marker='o', color="red", label='d_61')
        plt.plot(t, d_26, marker='o', color="orange", label='d_62')
        plt.plot(t, d_46, marker='o', color="yellow", label='d_64')
        plt.plot(t, d_56, marker='o', color="green", label='d_65')
        plt.plot(t, d_76, marker='o', color="blue", label='d_67')
        plt.plot(t, d_86, marker='o', color="purple", label='d_68')
        plt.plot(t, d_96, marker='o', color="brown", label='d_69')
        
        plt.plot(t, d_17, marker='o', color="red", label='d_71')
        plt.plot(t, d_27, marker='o', color="orange", label='d_72')
        plt.plot(t, d_47, marker='o', color="yellow", label='d_74')
        plt.plot(t, d_57, marker='o', color="green", label='d_75')
        plt.plot(t, d_67, marker='o', color="blue", label='d_76')
        plt.plot(t, d_87, marker='o', color="purple", label='d_78')
        plt.plot(t, d_97, marker='o', color="brown", label='d_79')
        
        plt.plot(t, d_18, marker='o', color="red", label='d_81')
        plt.plot(t, d_28, marker='o', color="orange", label='d_82')
        plt.plot(t, d_48, marker='o', color="yellow", label='d_84')
        plt.plot(t, d_58, marker='o', color="green", label='d_85')
        plt.plot(t, d_68, marker='o', color="blue", label='d_86')
        plt.plot(t, d_78, marker='o', color="purple", label='d_87')
        plt.plot(t, d_98, marker='o', color="brown", label='d_89')
        
        plt.plot(t, d_19, marker='o', color="red", label='d_91')
        plt.plot(t, d_29, marker='o', color="orange", label='d_92')
        plt.plot(t, d_49, marker='o', color="yellow", label='d_94')
        plt.plot(t, d_59, marker='o', color="green", label='d_95')
        plt.plot(t, d_69, marker='o', color="blue", label='d_96')
        plt.plot(t, d_79, marker='o', color="purple", label='d_97')
        plt.plot(t, d_89, marker='o', color="brown", label='d_98')
        '''

        

#         plt.pause(0.001)
#         if t < 0.3:
#             plt.legend(bbox_to_anchor=(1.1, 1), loc='upper right', borderaxespad=0, fontsize=8)
#             #ax2.legend(bbox_to_anchor=(1.05, 1), loc='upper right', borderaxespad=0, fontsize=8)
#         if t > 300:
#             plt.savefig('D4'+str(t)+'.png')
#             exit()
# plt.show()


def listener():
    rospy.init_node('graph')
    graph = Graph()
    #graph.plot()
    rospy.spin()

if __name__ == '__main__':
    start = time.time()
    #fig = plt.figure()
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
# END ALL
