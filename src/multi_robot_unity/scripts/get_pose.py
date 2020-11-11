#!/usr/bin/env python3

import rospy

from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from std_msgs.msg import Float64MultiArray
import sys

import math
import numpy as np
import matplotlib.pyplot as plt
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

ranges = [(-20,20), (-20,-20), (20,-20), (20,20)]
targets = [[-10,-10], [10,10], [-10, 5]]
weights = [[0.6],[0.3], [0.1]]
varience = 3
numbers = 3000
FLAG = True
count = 0

def get_pose(name):
    set = GetModelStateRequest()
    set.model_name = name
    response = call(set)
    return response.pose

class Voronoi:
    def __init__(self):
        self.pub = rospy.Publisher('/controler', Float64MultiArray, queue_size=10)
        self.pub_centroid = rospy.Publisher('/centroid', Float64MultiArray, queue_size=10)

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
            
        def plot_Voronoi(ax, outer, poly_shapes, coords, new_coords): # plot voronoi diagram
            area_shape = Polygon(outer)
            plot_voronoi_polys_with_points_in_area(ax, area_shape, poly_shapes, coords,
                                                   poly_to_pt_assignments, points_color='black', # plot centroid as black square
                                                   points_markersize=40, voronoi_and_points_cmap=None,
                                                   plot_voronoi_opts={'alpha': 0.5})
            for centroid in new_centroids:
                c1 = centroid
                ax.plot(c1[0],c1[1], 'rs', markersize = 8) # plot inner drones as red circle

            for coord in new_coords:
                c2 = coord
                ax.plot(c2[0],c2[1], 'b+', markersize = 8) # plot target pos as b + 
            
            # for vertex in outer:
            #     v1 = vertex
            #     ax.plot(v1[0],v1[1], c='yellow', marker='s', markersize = 8) # plot outer drones as yellow circle
            
            font = {'size':20}
            ax.set_xlim([min(ranges[:])[0],max(ranges[:])[0]]) # set range of x-axis
            ax.set_ylim([min(ranges[:])[1],max(ranges[:])[0]]) # set range of x-axis
            # ax.set_xlim([min(outer),400]) # set range of x-axis
            # ax.set_ylim([-30,400]) # set range of y-axis
            ax.xaxis.set_major_locator(major_locator) # set grid
            ax.yaxis.set_major_locator(major_locator)
            ax.set_xlabel('x(m)', font, labelpad=10) # set x-label
            ax.set_ylabel('y(m)', font, labelpad=10)
            ax.tick_params(labelsize=18)

        def plot_density(ax, x, y, norm):
            global ranges, targets
            # plot samples as scatter density plot
            # ax.scatter_density(x, y, cmap='tab20', norm=norm, alpha = 1.0)

            # plot samples as normal scatter plot (you can see each group colored different colors)
            color = ['+r', '+g', '+b', '+y', '+k']
            for i in range(len(targets)):
                if i==0:
                    ax.plot(x[:int(numbers*weights[i][0])], y[:int(numbers*weights[i][0])], color[i], alpha=0.5)
                    prev = int(numbers*weights[i][0])
                else:
                    ax.plot(x[prev:prev+int(numbers*weights[i][0])], y[prev:prev+int(numbers*weights[i][0])], color[i], alpha=0.5)
                    prev += int(numbers*weights[i][0])
            # ax.plot(x[int(2*numbers/4.0):int(3*numbers/4.0)], y[int(2*numbers/4.0):int(3*numbers/4.0)], '+b', alpha=0.5)
            # ax.plot(x[int(3*numbers/4.0):], y[int(3*numbers/4.0):], '+y', alpha=0.5)

            font = {'size':20}
            ax.set_xlim([min(ranges[:])[0],max(ranges[:])[0]])
            ax.set_ylim([min(ranges[:])[1],max(ranges[:])[0]])
            #ax.plot(target[0],target[1], 'rs', markersize = 8)
            ax.xaxis.set_major_locator(major_locator)
            ax.yaxis.set_major_locator(major_locator)
            ax.set_xlabel('x(m)',font, labelpad=10)
            ax.set_ylabel('y(m)',font, labelpad=10)
            ax.tick_params(labelsize=18)

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
            t=np.linspace(0,T,num=150) # in 2time step update 50 times  
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
            global ranges
            while len(pts)<N_POINTS:#isinstance(compensated, int):
                inner_pos = points_to_coords(pts)
                randx = np.random.uniform(min(ranges[:])[0], max(ranges[:])[0], N_POINTS-len(pts))
                randy = np.random.uniform(min(ranges[:])[1], max(ranges[:])[1], N_POINTS-len(pts))
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
                    x_unit = np.concatenate((x_unit,np.random.normal(targets[i+1][0], varience, int(numbers*weights[i+1][0]))))
                    y_unit = np.concatenate((y_unit,np.random.normal(targets[i+1][1], varience, int(numbers*weights[i+1][0]))))
                    
            # x_unit = np.random.normal(target[0], deviation, numbers)
            # y_unit = np.random.normal(target[1], deviation, numbers)
            return x_unit, y_unit



        global FLAG, fig, ax1, ax2, norm, major_locator, area_shape, poly_shapes, poly_to_pt_assignments, \
                new_centroids, ranges, targets, weights, varience, numbers, count, x_unit, y_unit, region_value

        ################Initialization#############
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

        # else:
        #     for target in targets:
        #         target[0] += np.random.randint(-1, 1)
        #         target[1] += np.random.randint(-1, 1)
        outer_pos = ranges

        # print(ranges, inner_pos, targets, weights, varience, numbers)
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

        for i, target in enumerate(targets):
            if i==0:
                target[0] += 3 * np.cos(count) #random.randint(-2, 2)
                target[1] += 3 * np.sin(count) #random.randint(-2, 2)
            if i==1:
                target[0] -= 3 * np.cos(count)
                target[1] -= 3 * np.sin(count)
            if i==2:
                target[0] += 3 * np.cos(count)
                target[1] -= 3 * np.sin(count)
        x_unit, y_unit = sampling(poly_shapes, targets, varience, numbers)
            
        

        ############pair points and centroids######
        # coords_ = reshape_coords(coords)
        region_value = value_contribution(x_unit, y_unit, poly_shapes)
        poly_centroids = centroid_calculation(region_value, poly_shapes)
        new_centroids = match_pair(poly_shapes, list(coords), list(poly_centroids))
        new_coords = cal_tra(coords,new_centroids)
        new_coords_ = coords_to_points(new_coords)

        # plot Voronoi
        plot_Voronoi(ax1, outer_pos, poly_shapes, coords, new_coords)
        # plot density
        plot_density(ax2, x_unit, y_unit, norm)

        #plt.title(str(j+1)  +"th itr")
        #plt.savefig('dense_images/FIG_'+str(j+1)+'.png')
        plt.pause(0.001)
        ax1.clear()
        ax2.clear()
        ###########trajactory calculation##########
        new_coords = np.array(new_coords).reshape(-1)
        print("UPDATED!  ", new_coords)
        data2send = Float64MultiArray(data = new_coords)
        self.pub.publish(data2send)
        centroid2send = Float64MultiArray(data = new_centroids)
        self.pub_centroid.publish(centroid2send)
        count += 1
        # return new_coords, new_centroids
        # else:
        #     plot_Voronoi(ax1, outer_pos, poly_shapes, inner_pos)
        
        

if __name__ == '__main__':

    rospy.init_node('view_node')

    call = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    # human_number = 5

    # name_list = []
    # # name_list.append("ubiquitous_display")

    # for i in range(human_number):
    #     name_list.append("actor" + str(i))


    rate = rospy.Rate(5)
    name_list = ['drone01', 'drone02', 'drone03', 'drone04', 'drone05', 'drone06', 'drone07', 'drone08', 'drone09']
    voronoi = Voronoi()
    while not rospy.is_shutdown():
        # pose = get_pose("drone01")
        # print(pose)
        # x = pose.position.x
        # y = pose.position.y
        # plt.plot(int(y),int(x), '^', color="red")

        pos_x = []
        pos_y = []
        for n in name_list:
            pose = get_pose(n)
            x = pose.position.x
            y = pose.position.y
            pos_x.append(x)
            pos_y.append(y)
            # plt.plot(int(y),int(x), 'o', color="blue")
        pos = np.vstack((np.array(pos_x).astype(np.float64), np.array(pos_y).astype(np.float64))).T
        # print (pos)
        voronoi.update(pos)
        # plt.draw()
        # plt.pause(0.1)
        # plt.clf()
        rate.sleep()