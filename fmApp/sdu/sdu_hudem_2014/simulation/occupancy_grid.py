#!/usr/bin/env python

## PARAMETERS ################################################################
PoseLogFile = 'sim_pose.txt'
PoseLogSkipLines = 0
PoseLogMaxLines = 8000000 # maximum number of lines to read from the log file
GPSLogFile = 'sim_gnss.txt'
GPSLogSkipLines = 0
GPSLogMaxLines = 800000 # maximum number of lines to read from the log file
HDOPThreshold = 10. # GPS fix above this HDOP value will be ignored
OrigoE = 588772 # subtract from all Easting coordinates
OrigoN = 6137285 # subtract from all Northing coordinates
TrackMinDist = 0.02 # [m] minimum distance between consequtive track points
HeadingMinDist = 0.25 # [m] minimum distance between track points when calculating heading

wads_cdist = 1.66 # distance from GPS antenna to center of implement

UPEXLogFile = 'sim_wads.txt'
UPEXLines = 50000000
UPEXSampleDist = 0.04

figure_size = 6
grid_size = 300

simStepInt = 0.02 # simulation step interval [s]

simPlotUpdate = 4000 # number of steps between updating the plots
mapPlotBnd = 5.0 # border around map plots [m] 

GPS_E = 0
GPS_N = 1

## INITIALIZATION ############################################################
# load modules
import sys
import signal
import time
import numpy as np
import matplotlib.pyplot as plt
from numpy import *
from pylab import *
import csv
from scipy.interpolate import griddata


pi2 = 2*pi

# return signed difference between new and old angle
def angle_diff (angle_new, angle_old):
	diff = angle_new - angle_old
	while diff < -pi:
		diff += pi2
	while diff > pi:
		diff -= pi2
	return diff


## SIMULATION REFERENCE DATA #################################################

# site polygon
#poly = [[588813.1026,6137314.1685],[588812.3040,6137299.0961],[588827.1685,6137298.0620],[588828.0036,6137312.9438]]
poly = [[588781.593,6137308.257],[588772.466,6137308.735],[588771.855,6137288.985],[588781.105,6137288.633]]
poly.append(list(poly[0])) # without a typecast this will create a reference, not a copy
for i in range(len(poly)):
	poly[i][0] -= OrigoE
	poly[i][1] -= OrigoN
polyT = zip (*poly)

# ground truth target list
gtt = [[588778.9326,6137291.6141],[588773.3388,6137293.7092],[588776.2196,6137299.5903],[588775.1004,6137303.9440],[588778.7879,6137303.6453],[588779.7928,6137296.2574]]

for i in range(len(gtt)):
	gtt[i][0] -= OrigoE
	gtt[i][1] -= OrigoN
gttT = zip (*gtt)

def gpsBoundaries(gpsData):
	minE = float(1e1000)
	minN = float(1e1000)
	maxE = -float(1e1000)
	maxN = -float(1e1000)
	for i in range(len(gpsData)):
		if minE > gpsData[i][GPS_E]:
			minE = gpsData[i][GPS_E]
		elif maxE < gpsData[i][GPS_E]:
			maxE = gpsData[i][GPS_E]
		if minN > gpsData[i][GPS_N]:
			minN = gpsData[i][GPS_N]
		elif maxN < gpsData[i][GPS_N]:
			maxN = gpsData[i][GPS_N]
	return (minE-mapPlotBnd, maxE+mapPlotBnd, minN-mapPlotBnd, maxN+mapPlotBnd)

def ptInPoly(e,n,p):
# http://www.ariel.com.au/a/python-point-int-poly.html
	l = len(p)
	inside = 0  
	p1e,p1n = p[0]
	for i in range(l+1):
		p2e,p2n = p[i % l]
		if n > min(p1n,p2n):
			if n <= max(p1n,p2n):
				if e <= max(p1e,p2e):
					if p1n != p2n:
						einters = (n-p1n)*(p2e-p1e)/(p2n-p1n)+p1e
					if p1e == p2e or e <= einters:
						inside = not inside
		p1e,p1n = p2e,p2n
	return inside

def import_occupancy_grid (filename):
	file = open(filename, 'r')
	data = csv.reader(file, delimiter=' ')
	l = []
	for e, n, wads in data:
		l.append([float(e)-OrigoE, float(n)-OrigoN, float(wads)])
	file.close()
	return (l)

grid = import_occupancy_grid ('occupancy_grid.csv')


(minE, maxE, minN, maxN) = gpsBoundaries(grid)
gridT = zip(*grid)


# extract columns
x = list(gridT[0])
y = list(gridT[1])
z = list(gridT[2])

plt.figure(num=1, figsize=(figure_size,figure_size), dpi=80, facecolor='w', edgecolor='w')

# define grid.
xi = np.linspace(minE,maxE,grid_size)
yi = np.linspace(minN,maxN,grid_size)
zi = griddata((x, y), z, (xi[None,:], yi[:,None]), method='linear')
# nearest eturn the value at the data point closest to the point of interpolation. See NearestNDInterpolator for more details.
# linear esselate the input point set to n-dimensional simplices, and interpolate linearly on each simplex. See LinearNDInterpolator for more details.
# cubic return the value determined from a piecewise cubic, continuously differentiable (C1), and approximately curvature-minimizing polynomial surface. See CloughTocher2DInterpolator for more details.

# contour the gridded data, plotting dots at the randomly spaced data points.

# plot kanten
#CS = plt.contour(xi,yi,zi,10,linewidths=0.5,colors=('0.5','0.4','0.3'))

#			CS = plt.contour(xi,yi,zi,15,linewidths=0.5,colors='k')
CS = plt.contourf(xi,yi,zi,10, cmap=plt.cm.YlOrRd)
plt.colorbar() # draw colorbar
plt.scatter(gttT[0],gttT[1],marker='x',c='b',s=35, lw = 1.5)

plt.savefig('plot_search_map.png')


##############################################################################
