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

# Parameters
KP = 5.0  # attractive potential gain
ETA = 100.0  # repulsive potential gain
AREA_WIDTH = 60.0  # potential area width [m]

def calc_potential_field(gx, gy, ox, oy, reso, rr):
    minx = ox - AREA_WIDTH / 2.0
    miny = oy - AREA_WIDTH / 2.0
    maxx = ox + AREA_WIDTH / 2.0
    maxy = oy + AREA_WIDTH / 2.0
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))

    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    for ix in range(xw):
        x = ix * reso + minx

        for iy in range(yw):
            y = iy * reso + miny
            ug = calc_attractive_potential(x, y, gx, gy)
            uo = calc_repulsive_potential(x, y, ox, oy, rr)
            uf = ug + uo
            pmap[ix][iy] = uf

    return pmap, minx, miny


def calc_attractive_potential(x, y, gx, gy):
    return 0.5 * KP * np.hypot(x - gx, y - gy)


def calc_repulsive_potential(x, y, ox, oy, rr):
    # calc repulsive potential
    dq = np.hypot(x - ox, y - oy)

    if dq <= rr:
        if dq <= 0.1:
            dq = 0.1

        return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
    else:
        return 0.0

def get_motion_model():
    # dx, dy
    motion = [[1, 0],
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]]

    return motion


def potential_field_planning(sx, sy, gx, gy, ox, oy, reso, rr):

    # calc potential field
    pmap, minx, miny = calc_potential_field(gx, gy, ox, oy, reso, rr)

    # search path
    d = np.hypot(sx - gx, sy - gy)
    ix = round((sx - minx) / reso)
    iy = round((sy - miny) / reso)
    gix = round((gx - minx) / reso)
    giy = round((gy - miny) / reso)

    motion = get_motion_model()
    minp = float("inf")
    minix, miniy = -1, -1
    for i, _ in enumerate(motion):
        inx = int(ix + motion[i][0])
        iny = int(iy + motion[i][1])
        if inx >= len(pmap) or iny >= len(pmap[0]):
            p = float("inf")  # outside area
        else:
            p = pmap[inx][iny]
        if minp > p:
            minp = p
            minix = inx
            miniy = iny
    ix = minix
    iy = miniy
    xp = ix * reso + minx
    yp = iy * reso + miny

    return xp, yp


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + '%s', data.twist[3])
    pub1 = rospy.Publisher('drone1/cmd_vel',Twist,queue_size=10)
    pub2 = rospy.Publisher('drone2/cmd_vel',Twist,queue_size=10)
    #rospy.loginfo('%s', data.pose.pose.position.x)
    print("potential_field_planning start")
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

    print("potential_field_planning start")
    sx = d1_posX
    sy = d1_posY
    gx = t_posX
    gy = t_posY
    grid_size = 0.5
    robot_radius = 5.0
    ox = d2_posX
    oy = d2_posY

    newx, newy = potential_field_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_radius)
    
    new_velx_d2 = t_velLin.x + 3*(t_posX-d2_posX)
    new_vely_d2 = t_velLin.y + 3*(t_posY-d2_posY)
    new_velx_d1 = (newx-d1_posX)
    new_vely_d1 = (newy-d1_posY)
    
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
