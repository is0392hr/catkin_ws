#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: 10969theodore, Koki
"""
import math
import random
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from matplotlib.pyplot import MultipleLocator
import numpy as np
from shapely.geometry import Polygon
from shapely.geometry import Point
from geovoronoi import coords_to_points, points_to_coords, voronoi_regions_from_coords
from geovoronoi.plotting import plot_voronoi_polys_with_points_in_area
import mpl_scatter_density
from astropy.visualization import LogStretch
from astropy.visualization.mpl_normalize import ImageNormalize

import tf
import rospy
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelStateRequest, GetModelState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

import time
import configparser

import threading

# ranges, targets, weights = [], [], []
# ranges = [[-800,680],[1080,680],[-800,-600],[1080, -600]]
ranges = [(-20,20), (-20,-20), (20,-20), (20,20)]
targets = [[0,0], [10,10]]
weights = [[0.9],[0.1]]
varience = 1
numbers = 1000
count = 0
pos = 0
############plot setting###################
# fig = plt.figure(figsize=(13,5))
# fig.subplots_adjust(wspace=0.3, hspace=0, top=0.9, bottom=0.2)
# ax1 = fig.add_subplot(121,projection='scatter_density') 
# ax2 = fig.add_subplot(122,projection='scatter_density')
# ax1.axis('scaled')
# ax2.axis('scaled')
# norm = ImageNormalize(vmin=0., vmax=1000, stretch=LogStretch())
# major_locator=MultipleLocator(100)

FLAG = True

def get_params():
    config = configparser.ConfigParser()
    config.read('/home/koki/catkin_ws/src/multi_robot_unity/scripts/index.txt')
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
    def __init__(self, name_list):
        self.data = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState) # rospy.Subscriber('/gazebo/get_model_states', GetModelState, self.callback)
        self.pub = rospy.Publisher('controler', Float64MultiArray, queue_size=32)
        self.pub_centroid = rospy.Publisher('centroid', Float64MultiArray, queue_size=32)
        self.name_list = name_list
        # self.ranges = rospy.Subscriber('/unity_ranges', PoseStamped, self.ranges_callback)
        # self.targets = rospy.Subscriber('/unity_targets', PoseStamped, self.targets_callback)
        # self.weights = rospy.Subscriber('/unity_weights', PoseStamped, self.weights_callback)
        # self.voronoi = Voronoi()
        
    # def ranges_callback(self, msg):
    #     global ranges
    #     rangeLT = msg.pose.position
    # def targets_callback(self, msg):
    #     global targets
    #     targets = msg.pose.position
    # def weights_callback(self, msg):
    #     global weights
    #     weights = msg.pose.position

    def get_pose(self, name_list):
        set = GetModelStateRequest()
        set.model_name = name_list
        response = set.data(set)
        return response.pose
    def callback(self, data):
        # voronoi = self.voronoi

        ##Get drones pos
        # drone01 = data.pose[idx[0]]
        # drone01_vel = data.twist[idx[0]]
        # d01_posX = drone01.position.x
        # d01_posY = drone01.position.y
        # d01_posZ = drone01.position.z
        # d01_velX = drone01_vel.linear.x
        # d01_velY = drone01_vel.linear.y
        
        # drone02 = data.pose[idx[1]]
        # drone02_vel = data.twist[idx[1]]
        # d02_posX = drone02.position.x
        # d02_posY = drone02.position.y
        # d02_posZ = drone02.position.z
        # d02_velX = drone02_vel.linear.x
        # d02_velY = drone02_vel.linear.y

        # drone03 = data.pose[idx[2]]
        # drone03_vel = data.twist[idx[2]]
        # d03_posX = drone03.position.x
        # d03_posY = drone03.position.y
        # d03_posZ = drone03.position.z
        # d03_velX = drone03_vel.linear.x
        # d03_velY = drone03_vel.linear.y

        # drone04 = data.pose[idx[3]]
        # drone04_vel = data.twist[idx[3]]
        # d04_posX = drone04.position.x
        # d04_posY = drone04.position.y
        # d04_posZ = drone04.position.z
        # d04_velX = drone04_vel.linear.x
        # d04_velY = drone04_vel.linear.y

        # drone05 = data.pose[idx[4]]
        # drone05_vel = data.twist[idx[4]]
        # d05_posX = drone05.position.x
        # d05_posY = drone05.position.y
        # d05_posZ = drone05.position.z
        # d05_velX = drone05_vel.linear.x
        # d05_velY = drone05_vel.linear.y

        pos_x = []
        pos_y = []
        for n in name_list:
            pose = self.get_pose(n)
            x = pose.position.x
            pos_x.append(x)
            y = pose.position.y
            pos_y.append(y)
            # plt.plot(int(y),int(x), 'o', color="blue")
        # global ranges, targets, weights, varience, numbers, count, coords, centroid
        
        #global inner_pos
        # print(ranges, targets, weights)
        # innerX = np.array([d01_posX, d02_posX, d03_posX, d04_posX, d05_posX]).astype(np.float64)
        # innerY = np.array([d01_posY, d02_posY, d03_posY, d04_posY, d05_posY]).astype(np.float64)
        global pos, count
        pos = np.vstack((np.array(pos_x).astype(np.float64), np.array(pos_y).astype(np.float64))).T
        print("inner: ", pos)

        if count%7==0:
            self.update(pos)
            pass
        
        # if count==0 or count%13==0:
        #     UPDATE = 1
        #     coords, centroid = voronoi.update(outer_pos=ranges, inner_pos=inner_pos, targets=targets, weights=weights, varience=varience, numbers=numbers, UPDATE=UPDATE)
        #     coords = np.array(coords).reshape(-1)
        #     print("UPDATED!  ", coords)
        #     data2send = Float64MultiArray(data = coords)
        #     self.pub.publish(data2send)
        #     centroid2send = Float64MultiArray(data = centroid)
        #     self.pub_centroid.publish(centroid2send)
        # else:
        #     UPDATE = 0   
        #     voronoi.update(outer_pos=ranges, inner_pos=inner_pos, targets=targets, weights=weights, varience=varience, numbers=numbers, UPDATE=UPDATE)
        
        count += 1
        rospy.sleep(3)
        # rospy.Rate(0.01)

    def update(self, inner_pos):
        global FLAG, fig, ax1, ax2, norm, major_locator #area_shape, poly_shapes, poly_to_pt_assignments, new_centroids
        if FLAG:
            fig = plt.figure(figsize=(13,5))
            fig.subplots_adjust(wspace=0.3, hspace=0, top=0.9, bottom=0.2)
            ax1 = fig.add_subplot(121,projection='scatter_density') 
            ax2 = fig.add_subplot(122,projection='scatter_density')
            ax1.axis('scaled')
            ax2.axis('scaled')
            norm = ImageNormalize(vmin=0., vmax=1000, stretch=LogStretch())
            major_locator=MultipleLocator(100)

            FLAG=False


        global ranges, targets, weights, varience, numbers
        
        outer_pos = ranges

        print(ranges, inner_pos, targets, weights, varience, numbers)
        # if UPDATE:
        N = len(inner_pos)
        # print("N: ", N)
        area_shape = Polygon(outer_pos)

        centroid_poly = np.array(area_shape.centroid.xy).astype(np.float64)
        pts = [p for p in coords_to_points(inner_pos) if p.within(area_shape)]  # converts to shapely Point
        pts = self.compensate(pts, N)

        coords = points_to_coords(pts)   # convert back to simple NumPy coordinate array
        poly_shapes, pts, poly_to_pt_assignments = voronoi_regions_from_coords(coords, area_shape, accept_n_coord_duplicates= 0)
        
        poly_centroids = np.array([p.centroid.coords[0] for p in poly_shapes])

        poly_shapes, pts, poly_to_pt_assignments = voronoi_regions_from_coords(coords, area_shape, accept_n_coord_duplicates= 0)

        ################Initialization#############
        x_unit, y_unit = self.sampling(N, poly_shapes, targets, varience, numbers)
        region_value = self.value_contribution(x_unit, y_unit, poly_shapes)
        poly_centroids = self.centroid_calculation(region_value, poly_shapes)

        ############pair points and centroids######
        coords_ = self.reshape_coords(coords, poly_shapes)
        new_centroids = self.match_pair(poly_shapes, coords_, poly_centroids)
        new_coords = self.cal_tra(coords,new_centroids)
        new_coords_ = coords_to_points(new_coords)
        #plot Voronoi
        self.plot_Voronoi(ax1, outer_pos, poly_shapes, coords, new_centroids, poly_to_pt_assignments)
        #plot density
        self.plot_density(ax2, x_unit, y_unit, norm)

        #plt.title(str(j+1)  +"th itr")
        #plt.savefig('dense_images/FIG_'+str(j+1)+'.png')
        plt.pause(0.1)
        ax1.clear()
        ax2.clear()
        while not rospy.is_shutdown():
            print("publishing")
            ###########trajactory calculation##########
            print('voronoi: ', new_coords)
            coords = np.array(new_coords).reshape(-1)
            print("UPDATED!  ", coords)
            data2send = Float64MultiArray(data = coords)
            self.pub.publish(data2send)
            centroid2send = Float64MultiArray(data = new_centroids)
            self.pub_centroid.publish(centroid2send)
            rospy.sleep(2.0)
        

    def reshape_coords(self, coords, poly_shapes): # find the corresponding point_centroid pairs
        new_coords = []
        for p in poly_shapes:
            for n in coords:
                m = Point(n)
                if m.within(p):
                    new_coords.append(n)
        return new_coords

    def match_pair(self, poly_shapes, coords, new_centroids): # sort order of centroid list to find pair of each region and its centroid
        sorted_centroids = []
        points = coords_to_points(coords)
        for i, p in enumerate(points):
            for j, poly in enumerate(poly_shapes):
                if p.within(poly): 
                    pair = new_centroids[j]
                    sorted_centroids.append(pair)
        return sorted_centroids

    def value_contribution(self, x_unit, y_unit, poly_shapes):
        point_value = np.vstack((np.array(x_unit), np.array(y_unit))).T # make pairs of samples
        poly_nums = len(poly_shapes)
        region_value =[[] for i in range(poly_nums)] #this stores coordinates of samples in a region
        for i, p in enumerate(point_value):
            for j, poly in enumerate(poly_shapes):
                point = Point(p) # convert sample coordinate to Point
                if point.within(poly): # if the sample locates in the region polyshapes[j] then append this p to region_value[j]
                    region_value[j].append(p)
        return np.array(region_value)

    def centroid_calculation(self, region_value, poly_shapes):
        sum_value = []
        for i in range(len(poly_shapes)):
            init = [0,0]
            for j in region_value[i]:
                init += j
            sum_value.append(init)
        poly_centroids = []
        for i in range(len(poly_shapes)):
            poly_size = len(region_value[i])
            if poly_size == 0: # If number of sample in a region is 0 then use its centroid as the next target
                poly_centroids.append(poly_shapes[i].centroid.coords[0])
            else: # next target is the CM of samples in a region
                poly_dense_x = sum_value[i][0]/poly_size
                poly_dense_y = sum_value[i][1]/poly_size
                poly_centroids.append([poly_dense_x,poly_dense_y])
        return poly_centroids
        
    def plot_Voronoi(self, ax, outer, poly_shapes, new_coords, new_centroids, poly_to_pt_assignments): # plot voronoi diagram
        area_shape = Polygon(outer)
        plot_voronoi_polys_with_points_in_area(ax, area_shape, poly_shapes, new_coords,
                                               poly_to_pt_assignments, points_color='black', # plot centroid as black square
                                               points_markersize=40, voronoi_and_points_cmap=None,
                                               plot_voronoi_opts={'alpha': 0.5})
        for centroid in new_centroids:
            c1 = centroid
            ax.plot(c1[0],c1[1], 'rs', markersize = 8) # plot inner drones as red circle

        # for coord in new_coords:
        #     c2 = coord
        #     ax.plot(c2[0],c2[1], 'r+', markersize = 8) # plot inner drones as red circle
        
        # for vertex in outer:
        #     v1 = vertex
        #     ax.plot(v1[0],v1[1], c='yellow', marker='s', markersize = 8) # plot outer drones as yellow circle
        
        font = {'size':20}
        # ax.set_xlim([min(outer),400]) # set range of x-axis
        # ax.set_ylim([-30,400]) # set range of y-axis
        ax.xaxis.set_major_locator(major_locator) # set grid
        ax.yaxis.set_major_locator(major_locator)
        ax.set_xlabel('x(m)', font, labelpad=10) # set x-label
        ax.set_ylabel('y(m)', font, labelpad=10)
        ax.tick_params(labelsize=18)

    def plot_density(self, ax, x, y, norm):
        # plot samples as scatter density plot
        ax.scatter_density(x, y, cmap='tab20', norm=norm, alpha = 1.0)

        # plot samples as normal scatter plot (you can see each group colored different colors)
        # ax.plot(x[:int(numbers/2.0)], y[:int(numbers/2.0)], '+r', alpha=0.5)
        # ax.plot(x[int(numbers/2.0):int(2*numbers/2.0)], y[int(numbers/2.0):int(2*numbers/2.0)], '+g', alpha=0.5)
        # ax.plot(x[int(2*numbers/4.0):int(3*numbers/4.0)], y[int(2*numbers/4.0):int(3*numbers/4.0)], '+b', alpha=0.5)
        # ax.plot(x[int(3*numbers/4.0):], y[int(3*numbers/4.0):], '+y', alpha=0.5)

        font = {'size':20}
        # ax.set_xlim([-30,400])
        # ax.set_ylim([-30,400])
        #ax.plot(target[0],target[1], 'rs', markersize = 8)
        ax.xaxis.set_major_locator(major_locator)
        ax.yaxis.set_major_locator(major_locator)
        ax.set_xlabel('x(m)',font, labelpad=10)
        ax.set_ylabel('y(m)',font, labelpad=10)
        ax.tick_params(labelsize=18)

    

    def cal_tra(self, new_coords,new_centroids):
        def diff_equation_x(y_list,t,e,omega):
            ki = 25
            sum_phi = y_list[1] + e
            coef = 0
            for i in range(2,len(y_list)):
                if i%2 == 0:
                    coef = int(i/2)
                    sum_phi += (y_list[i] + e*math.cos(coef*omega*t)) * math.cos(coef*omega*t)
                else:
                    coef = int((i-1)/2)
                    sum_phi += (y_list[i] + e*math.sin(coef*omega*t)) * math.sin(coef*omega*t)
            
            result = []
            result.append(-ki*e-sum_phi + 20*math.cos(math.pi*t))
            result.append(-e)
            for i in range(2,len(y_list)):
                if i%2 == 0:
                    coef = int(i/2)
                    result.append(coef*e*omega*math.sin(coef*omega*t) + ki*e*math.cos(coef*omega*t))
                else:
                    coef = int((i-1)/2)
                    result.append((-e)*coef*omega*math.cos(coef*omega*t) + ki*e*math.sin(coef*omega*t))
            return np.array(result)
                             
        def diff_equation_y(y_list,t,e,omega):
            ki = 25
            sum_fu = y_list[1] + e
            coef = 0
            for i in range(2,len(y_list)):
                if i%2 == 0:
                    coef = int(i/2)
                    sum_fu += (y_list[i] + e*math.cos(coef*omega*t)) * math.cos(coef*omega*t)
                else:
                    coef = int((i-1)/2)
                    sum_fu += (y_list[i] + e*math.sin(coef*omega*t)) * math.sin(coef*omega*t)
            result = []
            result.append(-ki*e-sum_fu + 20*math.sin(math.pi*t))
            result.append(-e)
            for i in range(2,len(y_list)):
                if i%2 == 0:
                    coef = int(i/2)
                    result.append(coef*e*omega*math.sin(coef*omega*t) + ki*e*math.cos(coef*omega*t))
                else:
                    coef = int((i-1)/2)
                    result.append((-e)*coef*omega*math.cos(coef*omega*t) + ki*e*math.sin(coef*omega*t))
            return np.array(result)
        T=2
        t=np.linspace(0,T,num=50) # in 2time step update 50 times  
        omega = math.pi*2/T
        point_lists = []
        for i in range(len(new_coords)):
            y_list_x = [new_coords[i][0],0,0,0,0,0,0,0,0,0,0]
            y_list_y = [new_coords[i][1],0,0,0,0,0,0,0,0,0,0]
            result_x = odeint(diff_equation_x, y_list_x, t, args=(new_coords[i][0]-new_centroids[i][0],omega))
            result_y = odeint(diff_equation_y, y_list_y, t, args=(new_coords[i][1]-new_centroids[i][1],omega))
            result_xt = result_x[:,0]
            result_yt = result_y[:,0]
            new_result = np.vstack((np.array(result_xt), np.array(result_yt))).T
            point_lists.append(list(new_result)[1])
        return point_lists

    def compensate(self, pts, N_POINTS):
        while len(pts)<N_POINTS:#isinstance(compensated, int):
            inner_pos = points_to_coords(pts)
            randx = np.random.uniform(minx, maxx, N_POINTS-len(pts))
            randy = np.random.uniform(miny, maxy, N_POINTS-len(pts))
            compensated = np.vstack((randx, randy)).T
            if len(inner_pos):
                inner_pos = np.append(inner_pos, compensated, axis=0)
            else:
                inner_pos = compensated
                #print(inner_pos)
                #inner_pos = inner_pos[sorted(np.random.choice(inner_pos.shape[0], N, replace=False)), :]
            pts = [p for p in coords_to_points(inner_pos) if p.within(area_shape)]  # converts to shapely Point
            print('%d of %d drone"s pos is available' % (len(pts), N_POINTS))
        return pts

    def sampling(self, N, polyshapes, targets, varience=20, numbers=1000):
        ####calculate centroid########
        ###########density distribution and centroids calculation#############
        frames=np.linspace(0,4*np.pi,N, endpoint=True) # for circumference circle movement
        # x_unit = np.random.normal(target[0], deviation, numbers)
        # y_unit = np.random.normal(target[1], deviation, numbers)
        # region_value = value_contribution(x_unit, y_unit, poly_shapes)
        if len(targets)==1:
            x_unit = np.random.normal(targets[0][0], varience, int(numbers*weights[0][0]))
            y_unit = np.random.normal(targets[0][1], varience, int(numbers*weights[0][0]))
        elif len(targets)>1:
            x_unit = np.random.normal(targets[0][0], varience, int(numbers*weights[0][0]))
            y_unit = np.random.normal(targets[0][1], varience, int(numbers*weights[0][0]))
            for i in range(len(targets)-1):
                x_unit = np.concatenate((x_unit,np.random.normal(targets[i+1][0], varience, int(numbers*weights[0][i]))))
                y_unit = np.concatenate((y_unit,np.random.normal(targets[i+1][1], varience, int(numbers*weights[0][i]))))
                
        # x_unit = np.random.normal(target[0], deviation, numbers)
        # y_unit = np.random.normal(target[1], deviation, numbers)
        return x_unit, y_unit



    


class Voronoi:
    def __init__(self):
        pass

    def update(self, inner_pos):

        def reshape_coords(coords): # find the corresponding point_centroid pairs
            new_coords = []
            for p in poly_shapes:
                for n in coords:
                    m = Point(n)
                    if m.within(p):
                        new_coords.append(n)
            return new_coords

        def match_pair(poly_shapes, coords, new_centroids): # sort order of centroid list to find pair of each region and its centroid
            sorted_centroids = []
            points = coords_to_points(coords)
            for i, p in enumerate(points):
                for j, poly in enumerate(poly_shapes):
                    if p.within(poly): 
                        pair = new_centroids[j]
                        sorted_centroids.append(pair)
            return sorted_centroids

        def value_contribution(x_unit, y_unit, poly_shapes):
            point_value = np.vstack((np.array(x_unit), np.array(y_unit))).T # make pairs of samples
            poly_nums = len(poly_shapes)
            region_value =[[] for i in range(poly_nums)] #this stores coordinates of samples in a region
            for i, p in enumerate(point_value):
                for j, poly in enumerate(poly_shapes):
                    point = Point(p) # convert sample coordinate to Point
                    if point.within(poly): # if the sample locates in the region polyshapes[j] then append this p to region_value[j]
                        region_value[j].append(p)
            return np.array(region_value)

        def centroid_calculation(region_value, poly_shapes):
            sum_value = []
            for i in range(len(poly_shapes)):
                init = [0,0]
                for j in region_value[i]:
                    init += j
                sum_value.append(init)
            poly_centroids = []
            for i in range(len(poly_shapes)):
                poly_size = len(region_value[i])
                if poly_size == 0: # If number of sample in a region is 0 then use its centroid as the next target
                    poly_centroids.append(poly_shapes[i].centroid.coords[0])
                else: # next target is the CM of samples in a region
                    poly_dense_x = sum_value[i][0]/poly_size
                    poly_dense_y = sum_value[i][1]/poly_size
                    poly_centroids.append([poly_dense_x,poly_dense_y])
            return poly_centroids
            
        # def plot_Voronoi(ax, outer, poly_shapes, new_coords): # plot voronoi diagram
        #     area_shape = Polygon(outer)
        #     plot_voronoi_polys_with_points_in_area(ax, area_shape, poly_shapes, new_coords,
        #                                            poly_to_pt_assignments, points_color='black', # plot centroid as black square
        #                                            points_markersize=40, voronoi_and_points_cmap=None,
        #                                            plot_voronoi_opts={'alpha': 0.5})
        #     for centroid in new_centroids:
        #         c1 = centroid
        #         ax.plot(c1[0],c1[1], 'rs', markersize = 8) # plot inner drones as red circle

        #     # for coord in new_coords:
        #     #     c2 = coord
        #     #     ax.plot(c2[0],c2[1], 'r+', markersize = 8) # plot inner drones as red circle
            
        #     # for vertex in outer:
        #     #     v1 = vertex
        #     #     ax.plot(v1[0],v1[1], c='yellow', marker='s', markersize = 8) # plot outer drones as yellow circle
            
        #     font = {'size':20}
        #     # ax.set_xlim([min(outer),400]) # set range of x-axis
        #     # ax.set_ylim([-30,400]) # set range of y-axis
        #     ax.xaxis.set_major_locator(major_locator) # set grid
        #     ax.yaxis.set_major_locator(major_locator)
        #     ax.set_xlabel('x(m)', font, labelpad=10) # set x-label
        #     ax.set_ylabel('y(m)', font, labelpad=10)
        #     ax.tick_params(labelsize=18)

        # def plot_density(ax, x, y, norm):
        #     # plot samples as scatter density plot
        #     ax.scatter_density(x, y, cmap='tab20', norm=norm, alpha = 1.0)

            # plot samples as normal scatter plot (you can see each group colored different colors)
            # ax.plot(x[:int(numbers/2.0)], y[:int(numbers/2.0)], '+r', alpha=0.5)
            # ax.plot(x[int(numbers/2.0):int(2*numbers/2.0)], y[int(numbers/2.0):int(2*numbers/2.0)], '+g', alpha=0.5)
            # ax.plot(x[int(2*numbers/4.0):int(3*numbers/4.0)], y[int(2*numbers/4.0):int(3*numbers/4.0)], '+b', alpha=0.5)
            # ax.plot(x[int(3*numbers/4.0):], y[int(3*numbers/4.0):], '+y', alpha=0.5)

            # font = {'size':20}
            # # ax.set_xlim([-30,400])
            # # ax.set_ylim([-30,400])
            # #ax.plot(target[0],target[1], 'rs', markersize = 8)
            # ax.xaxis.set_major_locator(major_locator)
            # ax.yaxis.set_major_locator(major_locator)
            # ax.set_xlabel('x(m)',font, labelpad=10)
            # ax.set_ylabel('y(m)',font, labelpad=10)
            # ax.tick_params(labelsize=18)

        def diff_equation_x(y_list,t,e,omega):
            ki = 25
            sum_phi = y_list[1] + e
            coef = 0
            for i in range(2,len(y_list)):
                if i%2 == 0:
                    coef = int(i/2)
                    sum_phi += (y_list[i] + e*math.cos(coef*omega*t)) * math.cos(coef*omega*t)
                else:
                    coef = int((i-1)/2)
                    sum_phi += (y_list[i] + e*math.sin(coef*omega*t)) * math.sin(coef*omega*t)
            
            result = []
            result.append(-ki*e-sum_phi + 20*math.cos(math.pi*t))
            result.append(-e)
            for i in range(2,len(y_list)):
                if i%2 == 0:
                    coef = int(i/2)
                    result.append(coef*e*omega*math.sin(coef*omega*t) + ki*e*math.cos(coef*omega*t))
                else:
                    coef = int((i-1)/2)
                    result.append((-e)*coef*omega*math.cos(coef*omega*t) + ki*e*math.sin(coef*omega*t))
            return np.array(result)
                             
        def diff_equation_y(y_list,t,e,omega):
            ki = 25
            sum_fu = y_list[1] + e
            coef = 0
            for i in range(2,len(y_list)):
                if i%2 == 0:
                    coef = int(i/2)
                    sum_fu += (y_list[i] + e*math.cos(coef*omega*t)) * math.cos(coef*omega*t)
                else:
                    coef = int((i-1)/2)
                    sum_fu += (y_list[i] + e*math.sin(coef*omega*t)) * math.sin(coef*omega*t)
            result = []
            result.append(-ki*e-sum_fu + 20*math.sin(math.pi*t))
            result.append(-e)
            for i in range(2,len(y_list)):
                if i%2 == 0:
                    coef = int(i/2)
                    result.append(coef*e*omega*math.sin(coef*omega*t) + ki*e*math.cos(coef*omega*t))
                else:
                    coef = int((i-1)/2)
                    result.append((-e)*coef*omega*math.cos(coef*omega*t) + ki*e*math.sin(coef*omega*t))
            return np.array(result)

        def cal_tra(new_coords,new_centroids):
            T=2
            t=np.linspace(0,T,num=50) # in 2time step update 50 times  
            omega = math.pi*2/T
            point_lists = []
            for i in range(len(new_coords)):
                y_list_x = [new_coords[i][0],0,0,0,0,0,0,0,0,0,0]
                y_list_y = [new_coords[i][1],0,0,0,0,0,0,0,0,0,0]
                result_x = odeint(diff_equation_x, y_list_x, t, args=(new_coords[i][0]-new_centroids[i][0],omega))
                result_y = odeint(diff_equation_y, y_list_y, t, args=(new_coords[i][1]-new_centroids[i][1],omega))
                result_xt = result_x[:,0]
                result_yt = result_y[:,0]
                new_result = np.vstack((np.array(result_xt), np.array(result_yt))).T
                point_lists.append(list(new_result)[1])
            return point_lists

        def compensate(pts, N_POINTS):
            while len(pts)<N_POINTS:#isinstance(compensated, int):
                inner_pos = points_to_coords(pts)
                randx = np.random.uniform(minx, maxx, N_POINTS-len(pts))
                randy = np.random.uniform(miny, maxy, N_POINTS-len(pts))
                compensated = np.vstack((randx, randy)).T
                if len(inner_pos):
                    inner_pos = np.append(inner_pos, compensated, axis=0)
                else:
                    inner_pos = compensated
                    #print(inner_pos)
                    #inner_pos = inner_pos[sorted(np.random.choice(inner_pos.shape[0], N, replace=False)), :]
                pts = [p for p in coords_to_points(inner_pos) if p.within(area_shape)]  # converts to shapely Point
                print('%d of %d drone"s pos is available' % (len(pts), N_POINTS))
            return pts

        def sampling(polyshapes, targets, varience=20, numbers=1000):
            ####calculate centroid########
            ###########density distribution and centroids calculation#############
            frames=np.linspace(0,4*np.pi,N, endpoint=True) # for circumference circle movement
            # x_unit = np.random.normal(target[0], deviation, numbers)
            # y_unit = np.random.normal(target[1], deviation, numbers)
            # region_value = value_contribution(x_unit, y_unit, poly_shapes)
            if len(targets)==1:
                x_unit = np.random.normal(targets[0][0], varience, int(numbers*weights[0][0]))
                y_unit = np.random.normal(targets[0][1], varience, int(numbers*weights[0][0]))
            elif len(targets)>1:
                x_unit = np.random.normal(targets[0][0], varience, int(numbers*weights[0][0]))
                y_unit = np.random.normal(targets[0][1], varience, int(numbers*weights[0][0]))
                for i in range(len(targets)-1):
                    x_unit = np.concatenate((x_unit,np.random.normal(targets[i+1][0], varience, int(numbers*weights[0][i]))))
                    y_unit = np.concatenate((y_unit,np.random.normal(targets[i+1][1], varience, int(numbers*weights[0][i]))))
                    
            # x_unit = np.random.normal(target[0], deviation, numbers)
            # y_unit = np.random.normal(target[1], deviation, numbers)
            return x_unit, y_unit



        # global FLAG, fig, ax1, ax2, norm, major_locator, area_shape, poly_shapes, poly_to_pt_assignments, new_centroids
        # if FLAG:
        #     fig = plt.figure(figsize=(13,5))
        #     fig.subplots_adjust(wspace=0.3, hspace=0, top=0.9, bottom=0.2)
        #     # ax1 = fig.add_subplot(121,projection='scatter_density') 
        #     ax2 = fig.add_subplot(122,projection='scatter_density')
        #     # ax1.axis('scaled')
        #     ax2.axis('scaled')
        #     norm = ImageNormalize(vmin=0., vmax=1000, stretch=LogStretch())
        #     major_locator=MultipleLocator(100)

        #     FLAG=False


        global ranges, targets, weights, varience, numbers
        
        outer_pos = ranges

        print(ranges, inner_pos, targets, weights, varience, numbers)
        # if UPDATE:
        N = len(inner_pos)
        # print("N: ", N)
        area_shape = Polygon(outer_pos)

        centroid_poly = np.array(area_shape.centroid.xy).astype(np.float64)
        pts = [p for p in coords_to_points(inner_pos) if p.within(area_shape)]  # converts to shapely Point
        pts = compensate(pts, N)

        coords = points_to_coords(pts)   # convert back to simple NumPy coordinate array
        poly_shapes, pts, poly_to_pt_assignments = voronoi_regions_from_coords(coords, area_shape, accept_n_coord_duplicates= 0)
        
        poly_centroids = np.array([p.centroid.coords[0] for p in poly_shapes])

        poly_shapes, pts, poly_to_pt_assignments = voronoi_regions_from_coords(coords, area_shape, accept_n_coord_duplicates= 0)

        ################Initialization#############
        x_unit, y_unit = sampling(poly_shapes, targets, varience, numbers)
        region_value = value_contribution(x_unit, y_unit, poly_shapes)
        poly_centroids = centroid_calculation(region_value, poly_shapes)

        ############pair points and centroids######
        coords_ = reshape_coords(coords)
        new_centroids = match_pair(poly_shapes, coords_, poly_centroids)
        new_coords = cal_tra(coords,new_centroids)
        new_coords_ = coords_to_points(new_coords)
        #plot Voronoi
        # plot_Voronoi(ax1, outer_pos, poly_shapes, coords)
        # #plot density
        # plot_density(ax2, x_unit, y_unit, norm)

        # #plt.title(str(j+1)  +"th itr")
        # #plt.savefig('dense_images/FIG_'+str(j+1)+'.png')
        # plt.pause(0.001)
        # ax1.clear()
        # ax2.clear()
        ###########trajactory calculation##########
        print('voronoi: ', new_coords)
        return new_coords, new_centroids
        # else:
        #     plot_Voronoi(ax1, outer_pos, poly_shapes, inner_pos)
        
        
              
        
    
def listener():
    rospy.init_node('Controler')
    
    # global pos
    r = rospy.Rate(0.3)
    name_list = ['drone01', 'drone02', 'drone03', 'drone04', 'drone05']
    controler = Controler(name_list)

    # while not rospy.is_shutdown():
    # thread = threading.Thread(target=controler)
    # thread.start()
    rospy.spin()
    # rospy.sleep(3)
        
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
