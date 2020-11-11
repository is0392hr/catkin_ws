#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Polygon
from shapely.geometry import Point
from geovoronoi import coords_to_points, points_to_coords, voronoi_regions_from_coords
from geovoronoi.plotting import subplot_for_map, plot_voronoi_polys_with_points_in_area
import math
import random 

import matplotlib.animation as animation

N_POINTS = 30 # number of inner drones

#np.random.seed(123)
outer = [(100, 5), (220, 5), (300, 50), (300, 240),(250, 300), (120,300), (5, 240)]
area_shape = Polygon(outer) # equal to outer drone pos

# generate some random points within the bounds
minx, miny, maxx, maxy = area_shape.bounds

randx = np.random.uniform(minx, maxx, N_POINTS)
randy = np.random.uniform(miny, maxy, N_POINTS)
coords = np.vstack((randx, randy)).T

pts = [p for p in coords_to_points(coords) if p.within(area_shape)]  # converts to shapely Point

print('will use %d of %d randomly generated points that are inside geographic area' % (len(pts), N_POINTS))
coords = points_to_coords(pts)   # convert back to simple NumPy coordinate array
del pts

#
# calculate the Voronoi regions, cut them with the geographic area shape and assign the points to them
#
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
                new_centroids.append(n)
    return new_centroids

def updayte_outer(outer):
    for i in range(len(outer)):
        x = int(outer[i][0])
        y = int(outer[i][1])
        x += random.randint(-10, 10)
        y += random.randint(-10, 10)

        outer[i] = (x,y)
    return outer

def find_pair(old_coords, new_coords):
    sorted_coords = []
    for old_coord in old_coords:
        min_dist = 99999999
        dist = 99999
        for i, new_coord in enumerate(new_coords):
            dist = math.sqrt((old_coord[0]-new_coord[0])**2 + (old_coord[1]-new_coord[1])**2)
            if min_dist > dist:
                min_dist = dist
                idx = i
        pair = new_coords.pop(idx)
        
    return sorted_coords

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
 
def AVG(lst): 
    return sum(lst) / len(lst)   

poly_shapes, pts, poly_to_pt_assignments = voronoi_regions_from_coords(coords, area_shape, accept_n_coord_duplicates= 0)
poly_centroids = np.array([p.centroid.coords[0] for p in poly_shapes])
new_centroids = reshape_centroids(poly_centroids)

# plotting
EPS = 0.1
err = 99999
itr = 1
fig, ax = subplot_for_map()
old_coords = coords
new_centroids = match_pair(poly_shapes, coords, new_centroids)#find_pair(coords, new_centroids)

total_error = []

while err > EPS:
    #coords = reshape_coords(coords)
    #old_coords = reshape_coords(old_coords)
    tmp_err = []
    if itr%1 == 0:
        # outer = updayte_outer(outer) # update outer position
        # area_shape = Polygon(outer)
        poly_shapes, pts, poly_to_pt_assignments = voronoi_regions_from_coords(coords, area_shape, accept_n_coord_duplicates= 0)
        poly_centroids = np.array([p.centroid.coords[0] for p in poly_shapes])
        new_centroids = reshape_centroids(poly_centroids)
        new_centroids = match_pair(poly_shapes, coords, new_centroids)#find_pair(coords, new_centroids)
    
    if itr != 1:
        for i in range(len(coords)):
            xo = coords[i][0]
            yo = coords[i][1]
            old_coords[i][0] = xo
            old_coords[i][1] = yo
            xc = new_centroids[i][0]
            yc = new_centroids[i][1]
            err = np.sqrt((xo-xc)**2 + (yo-yc)**2)
            
            tmp_err.append(err)

            if  err > EPS:
                # print("UPDARED!!")
                coords[i][0] = xc#xo + 0.2*(xc-xo)
                coords[i][1] = yc#yo + 0.2*(yc-yo)
    if itr%1==0:
        ax.set_title('%d th' % (itr))

        # draw centroid that each drone follow
        for centroid in new_centroids:
            c1 = centroid
            ax.plot(c1[0],c1[1], '*')
        for coord in old_coords:
            c = coord
            ax.plot(c[0],c[1], 'o', alpha = 0.5)

        fig = plot_voronoi_polys_with_points_in_area(ax, area_shape, poly_shapes, coords, poly_to_pt_assignments)
        plt.title(str(itr)  +"th itr")

        if err <= EPS or itr <= 30:
            print("FIG SAVED!")
            #plt.savefig('0'+str(itr)+'.png')
            if itr <10:
                plt.savefig('FIG_0'+str(itr)+'.png')
            else:
                plt.savefig('FIG_'+str(itr)+'.png')
        if err > EPS:
            plt.pause(0.0001)
            ax.clear() # comment out if you want to plot trajectory
        else:
            plt.show()

    if itr>1:
        tmp_err = AVG(tmp_err)
        total_error.append(tmp_err)
    print(itr)
    itr += 1

print(total_error)



for i,e in enumerate(total_error):
    plt.plot(i,e,'*-')
plt.ylabel('Error')
plt.xlabel('Iter')
plt.savefig('converge.png')
plt.show()

