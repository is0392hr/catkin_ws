#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Polygon
from shapely.geometry import Point
from geovoronoi import coords_to_points, points_to_coords, voronoi_regions_from_coords
from geovoronoi.plotting import subplot_for_map, plot_voronoi_polys_with_points_in_area
import math
import random 

import matplotlib.animation as animation

import tf
import rospy
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import Twist, Vector3

import time
import configparser

global FLAG
FLAG = True


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



class Controler:
    def __init__(self):
        self.data = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.pub = rospy.Publisher('controler', Float64MultiArray, queue_size=10)
        self.pub_centroid = rospy.Publisher('centroid', Float64MultiArray, queue_size=10)

    def callback(self, data):
        voronoi = Voronoi()

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

        drone03 = data.pose[idx[2]]
        drone03_vel = data.twist[idx[2]]
        d03_posX = drone03.position.x
        d03_posY = drone03.position.y
        d03_posZ = drone03.position.z
        d03_velX = drone03_vel.linear.x
        d03_velY = drone03_vel.linear.y

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

        drone10 = data.pose[idx[9]]
        drone10_vel = data.twist[idx[9]]
        d10_posX = drone10.position.x
        d10_posY = drone10.position.y
        d10_posZ = drone10.position.z
        d10_velX = drone10_vel.linear.x
        d10_velY = drone10_vel.linear.y

        drone11 = data.pose[idx[10]]
        drone11_vel = data.twist[idx[10]]
        d11_posX = drone11.position.x
        d11_posY = drone11.position.y
        d11_posZ = drone11.position.z
        d11_velX = drone11_vel.linear.x
        d11_velY = drone11_vel.linear.y

        drone12 = data.pose[idx[11]]
        drone12_vel = data.twist[idx[11]]
        d12_posX = drone12.position.x
        d12_posY = drone12.position.y
        d12_posZ = drone12.position.z
        d12_velX = drone12_vel.linear.x
        d12_velY = drone12_vel.linear.y

        drone13 = data.pose[idx[12]]
        drone13_vel = data.twist[idx[12]]
        d13_posX = drone13.position.x
        d13_posY = drone13.position.y
        d13_posZ = drone13.position.z
        d13_velX = drone13_vel.linear.x
        d13_velY = drone13_vel.linear.y

        drone14 = data.pose[idx[13]]
        drone14_vel = data.twist[idx[13]]
        d14_posX = drone14.position.x
        d14_posY = drone14.position.y
        d14_posZ = drone14.position.z
        d14_velX = drone14_vel.linear.x
        d14_velY = drone14_vel.linear.y

        drone15 = data.pose[idx[14]]
        drone15_vel = data.twist[idx[14]]
        d15_posX = drone15.position.x
        d15_posY = drone15.position.y
        d15_posZ = drone15.position.z
        d15_velX = drone15_vel.linear.x
        d15_velY = drone15_vel.linear.y

        drone16 = data.pose[idx[15]]
        drone16_vel = data.twist[idx[15]]
        d16_posX = drone16.position.x
        d16_posY = drone16.position.y
        d16_posZ = drone16.position.z
        d16_velX = drone16_vel.linear.x
        d16_velY = drone16_vel.linear.y

        drone17 = data.pose[idx[16]]
        drone17_vel = data.twist[idx[16]]
        d17_posX = drone17.position.x
        d17_posY = drone17.position.y
        d17_posZ = drone17.position.z
        d17_velX = drone17_vel.linear.x
        d17_velY = drone17_vel.linear.y
        #global t_updated, outer_pos, inner_pos
        
        outer_pos = [(d01_posX, d01_posY),
                    (d02_posX, d02_posY), 
                    (d04_posX, d04_posY), 
                    (d05_posX, d05_posY), 
                    (d06_posX, d06_posY),
                    (d07_posX, d07_posY),
                    (d08_posX, d08_posY),
                    (d09_posX, d09_posY)]

        innerX = np.array([d10_posX, d11_posX, d12_posX,  d13_posX, d14_posX, d15_posX, d16_posX, d17_posX, d03_posX]).astype(np.float64)
        innerY = np.array([d10_posY, d11_posY, d12_posY,  d13_posY, d14_posY, d15_posY, d16_posY, d17_posY, d03_posY]).astype(np.float64)

        inner_pos = np.vstack((innerX, innerY)).T

        UPDATE = True

        
           
        coords, centroid = voronoi.update(outer_pos=outer_pos, inner_pos=inner_pos, UPDATE=UPDATE)
        coords = np.array(coords).reshape(-1)
        print("UPDATED!  ", coords, centroid)

        data2send = Float64MultiArray(data = coords)
        self.pub.publish(data2send)
        centroid2send = Float64MultiArray(data = centroid)
        self.pub_centroid.publish(centroid2send)
        rospy.sleep(0.5)




class Voronoi:
    def __init__(self):

        pass     
    
    def update_outer(outer_pos):
        area_shape = Polygon(outer_pos) # equal to outer drone pos

        return area_shape

    def update_inner(inner_pos, area_shape):
        pts = [p for p in coords_to_points(inner_pos) if p.within(area_shape)]  # converts to shapely Point
        coords = points_to_coords(pts)   # convert back to simple NumPy coordinate array

        return coords
    

    def update(self, outer_pos, inner_pos, UPDATE, EPS = 0.1):
        global t
        t = time.time()-start

        outputs = []

        global FLAG
        if FLAG:
            fig, ax = subplot_for_map()
            global ax, fig
            FLAG=False

    
        def reshape_coords(coords):
            new_coords = []
            for p in poly_shapes:
                for n in coords:
                    m = Point(n)
                    if m.within(p):
                        new_coords.append(n)
            return new_coords

        def reshape_centroids(centroids):
            new_centroids = []
            for p in poly_shapes:
                for n in centroids:
                    m = Point(n)
                    if m.within(p):
                        new_cent
                        roids.append(n)
            return new_centroids

        def match_pair(poly_shapes, coords, new_centroids):
            sorted_coords = []
            points = coords_to_points(coords)
            for i, p in enumerate(points):
                c = coords[i]
                #print("c: ", c[0],c[1])
                for j, poly in enumerate(poly_shapes):
                    if p.within(poly):
                        pair = new_centroids[j]
                        sorted_coords.append(pair)
            return sorted_coords
        N = len(inner_pos)
        print("N: ", N)
        area_shape = Polygon(outer_pos) #update_outer(outer_pos)
        centroid_poly = np.array(area_shape.centroid.xy).astype(np.float64)
        

        pts = [p for p in coords_to_points(inner_pos) if p.within(area_shape)]  # converts to shapely Point
        
        while len(pts)<N:#isinstance(compensated, int):
            inner_pos = points_to_coords(pts)
            print('%d of %d drone"s pos is available' % (len(pts), N))
            # generate some random points within the bounds
            minx, miny, maxx, maxy = area_shape.bounds
            #print("compensated!!", compensated, type(compensated))
            
            randx = np.random.uniform(minx, maxx, N-len(pts))
            randy = np.random.uniform(miny, maxy, N-len(pts))
            compensated = np.vstack((randx, randy)).T
            if len(inner_pos):
                inner_pos = np.append(inner_pos, compensated, axis=0)
            else:
                inner_pos = compensated
            #print(inner_pos)
            #inner_pos = inner_pos[sorted(np.random.choice(inner_pos.shape[0], N, replace=False)), :]
            pts = [p for p in coords_to_points(inner_pos) if p.within(area_shape)]  # converts to shapely Point
            print('%d of %d drone"s pos is available' % (len(pts), N))
        
        ax.clear() # comment out if you want to plot trajectory
        coords = points_to_coords(pts)   # convert back to simple NumPy coordinate array
        poly_shapes, pts, poly_to_pt_assignments = voronoi_regions_from_coords(coords, area_shape, accept_n_coord_duplicates= 0)
        
        poly_centroids = np.array([p.centroid.coords[0] for p in poly_shapes])
        #new_centroids = reshape_centroids(poly_centroids)

        # plotting
        EPS = EPS
        err = 99999

        #old_coords = coords
        new_centroids = match_pair(poly_shapes, coords, poly_centroids)

    
        for i in range(len(coords)):
            xo = coords[i][0]
            yo = coords[i][1]
            #old_coords[i][0] = xo
            #old_coords[i][1] = yo
            xc = new_centroids[i][0]
            yc = new_centroids[i][1]
            #err = np.sqrt((xo-xc)**2 + (yo-yc)**2)
            
            data = [xc, yc]

            outputs.append(data)#(np.array((xc, yc)).astype(np.float64))

            #if  err > EPS:
            #    # print("UPDARED!!")
            #    coords[i][0] = xc#xo + 0.2*(xc-xo)
            #    coords[i][1] = yc#yo + 0.2*(yc-yo)


        # draw centroid that each drone follow
        for i, centroid in enumerate(new_centroids):
            c1 = centroid
            ax.plot(c1[0],c1[1], '*', label=str(i), color='red',markersize=15)
        for coord in coords:
            c = coord
            ax.plot(c[0],c[1], 'o', alpha = 0.5, color='blue', markersize=10)

        fig = plot_voronoi_polys_with_points_in_area(ax, area_shape, poly_shapes, coords, poly_to_pt_assignments)
        plt.title(str(t)+"[s]")
        
        plt.pause(0.00001)
        #time.sleep(3)
        return outputs, centroid_poly

        
        
     
def listener():
    rospy.init_node('Controler')
    controler = Controler()
    rospy.spin()

if __name__ == '__main__':
    start = time.time()
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
