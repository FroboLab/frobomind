#!/usr/bin/env python
#*****************************************************************************
# occupancy_grid_map
# Copyright (c) 2013, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name FroboMind nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#*****************************************************************************
"""
"""

# imports
import sys
import numpy as np
import matplotlib.pyplot as plt
from numpy import *
from pylab import *
import csv
from scipy.interpolate import griddata

# parameters
grid_file = 'occupancy_grid.csv'
origo_e = 588772 # subtract from all Easting coordinates
origo_n = 6137285 # subtract from all Northing coordinates
plot_canvas = 1.0 # m

poly = [[588781.593,6137308.257],[588772.466,6137308.735],[588771.855,6137288.985],[588781.105,6137288.633],[588781.593,6137308.257]]
gtt = [[588778.9326,6137291.6141],[588773.3388,6137293.7092],[588776.2196,6137299.5903],[588775.1004,6137303.9440],[588778.7879,6137303.6453],[588779.7928,6137296.2574]]

# defines
E = 0
N = 1
WADS = 2

def boundaries(gpsData):
    minE = float(1e1000)
    minN = float(1e1000)
    maxE = -float(1e1000)
    maxN = -float(1e1000)
    for i in range(len(gpsData)):
        if minE > gpsData[i][E]:
            minE = gpsData[i][E]
        elif maxE < gpsData[i][E]:
            maxE = gpsData[i][E]
        if minN > gpsData[i][N]:
            minN = gpsData[i][N]
        elif maxN < gpsData[i][N]:
            maxN = gpsData[i][N]
    return (minE-plot_canvas, maxE+plot_canvas, minN-plot_canvas, maxN+plot_canvas)

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

# prepare area polygon'poly.append(list(poly[0])) # without a typecast this will create a reference, not a copy
for i in range(len(poly)):
    poly[i][0] -= origo_e
    poly[i][1] -= origo_n
polyT = zip (*poly)

# prepare ground truth target list
for i in range(len(gtt)):
    gtt[i][0] -= origo_e
    gtt[i][1] -= origo_n
gttT = zip (*gtt)

#import data
f = open(grid_file, 'r')
data = csv.reader(f, delimiter=';')
g = []
for east, north, wads in data:
	e = float(east)-origo_e
	n = float(north)-origo_n
	if ptInPoly (e,n, poly):
	    g.append ([e, n, float(wads)])
f.close()
             
# extract columns
gT = zip (*g)
x = list(gT[0])
y = list(gT[1])
z = list(gT[2])

# define grid.
(minE, maxE, minN, maxN) = boundaries(g)
xi = np.linspace(minE,maxE,300)
yi = np.linspace(minN,maxN,300)
zi = griddata((x, y), z, (xi[None,:], yi[:,None]), method='cubic')

# setup plot
plt.figure(num=1, figsize=(8, 8), dpi=80, facecolor='w', edgecolor='w')
plt.clf() # clear figure
ax = gca()
#ax.set_frame_on(False)
#ax.axes.get_xaxis().set_visible(False)
#ax.axes.get_yaxis().set_visible(False)
#title ('Wads hazard map')
#xlabel('Easting [m]')
#ylabel('Northing [m]')
xlim([minE,maxE])
ylim([minN,maxN])
axis('equal')
grid (True)


# contour the gridded data, plotting dots at the randomly spaced data points.
CS = plt.contour(xi,yi,zi,15,linewidths=0.5,colors=('0.5','0.4','0.3'))
CS = plt.contourf(xi,yi,zi,15,cmap=plt.cm.YlOrRd)

polyPlt = plot(polyT[0], polyT[1],'g')
#targetPlt = plot(gttT[0], gttT[1],'bx')
plt.scatter(gttT[0],gttT[1],marker='x',c='b',s=35, lw = 1.5)

ion()
draw()
show()
plt.savefig ('plot_occ_grid.png')

#plt.figure(num=4, figsize=(8, 8), dpi=80, facecolor='w', edgecolor='w')
#heatmap, xedges, yedges = np.histogram2d(x, y, bins=(10, 20))
#extent = [minE, maxE, minN, maxN]

#plt.clf()
#plt.imshow(heatmap, extent=extent)
#plt.show()

#from matplotlib import pyplot as PLT
#from matplotlib import cm as CM
#from matplotlib import mlab as ML
#import numpy as NP

#print x
#plt.figure(num=5, figsize=(8, 8), dpi=80, facecolor='w', edgecolor='w')

#plt.hexbin(x,y, gridsize=100, cmap=plt.cm.YlOrRd_r)
#plt.axis([minE, maxE, minN, maxN])
#plt.title("Hexagon binning")
#cb = plt.colorbar()
#cb.set_label('counts')


raw_input() # wait for enter keypress 

 


