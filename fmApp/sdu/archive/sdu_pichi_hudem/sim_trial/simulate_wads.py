#!/usr/bin/env python

## PARAMETERS ################################################################
#GPSLogFile = 'gpslog/datagrid-20120417-142017_ex1.gga'
#GPSLogFile = 'gpslog/datagrid-20120417-150813_ex2.gga'
#GPSLogFile = 'gpslog/datagrid-20120417-154003_ex3.gga'
GPSLogFile = 'sim_pose.txt'
GPSLogSkipLines = 350
HDOPThreshold = 10. # GPS fix above this HDOP value will be ignored
GPSLines = 80000 # maximum number of lines to read from the log file
OrigoE = 588772 # subtract from all Easting coordinates
OrigoN = 6137285 # subtract from all Northing coordinates
TrackMinDist = 0.02 # [m] minimum distance between consequtive track points
HeadingMinDist = 0.20 # [m] minimum distance between track points when calculating heading

wads_cdist = 1.66 # distance from GPS antenna to center of implement

#UPEXLogFile = 'upex740log/upex740-20120417-142006_ex1.upex'
#UPEXLogFile = 'upex740log/upex740-20120417-150804_ex2.upex'
#UPEXLogFile = 'upex740log/upex740-20120417-153957_ex3.upex'
UPEXLogFile = 'sim_wads.txt'
UPEXLines = 50000
UPEXSampleDist = 0.05

simStepInt = 0.02 # simulation step interval [s]

simPlotUpdate = 10000 # number of steps between updating the plots
mapPlotBnd = 5.0 # border around map plots [m] 

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

# define imported GPS data columns
GPS_TIME = 0
GPS_E = 1
GPS_N = 2
GPS_YAW = 3

# define calculated GPS data columns
GPS_COURSE = 8

# define imported UPEX 740M data columns
UPEX_TIME = 0
UPEX_ADC = 1

# extra defines for the simulated UPEX 740M data
UPEX_N = 2
UPEX_E = 3

# define ctrl-c handler
Ctrl_C = 0

def signal_handler(signal, frame):
    global Ctrl_C
    Ctrl_C = 1
    print 'Ctrl-C pressed'
    print 'Quit'

# install ctrl-c handler
signal.signal(signal.SIGINT, signal_handler)

def vec2d_rot (v, theta): # return the vector v rotated by theta
	rot_x = v[0]*cos(theta) - v[1]*sin(theta)
	rot_y = v[0]*sin(theta) + v[1]*cos(theta)
	return ([rot_x, rot_y])

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

# wads_implement
wframe = [[-1.0,-0.5],[1.0,-0.5],[1.0,0.5],[-1.0,0.5],[-1.0,-0.5]]

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

## GPS #######################################################################
def importGPSLog (filename, hdop_threshold, max_lines):
    file = open(filename, 'rb')
    data = csv.reader(file, delimiter=',')
    cnt = 0
    i=0
    l = []
    samples_nofix = 0
    samples_above_hdop = 0
    for time, utme, utmn, yaw in data:
        cnt += 1	
        if cnt > GPSLogSkipLines: #int(fix) > 0:
            if True: #float(hdop) <= hdop_threshold:
                l.append([])
                l[i].append (float(time))
                l[i].append (float(utme)-OrigoE)
                l[i].append (float(utmn)-OrigoN)
                l[i].append (float(yaw))
                i += 1
            else:
                samples_above_hdop += 1
        else:
            samples_nofix += 1
        if i == max_lines: # break if maximum samples is reached
            break

    file.close()
    return (l,samples_nofix, samples_above_hdop)

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

## UPEX 740M #################################################################
def importUpex740Log (filename, max_lines):
    file = open(filename, 'rb')
    data = csv.reader(file, delimiter=',')
    i=0
    l = []
    samples_nofix = 0
    samples_above_hdop = 0
    for time, adc in data:
        l.append([])
        l[i].append (float(time))
        l[i].append (int(adc)*8./(1023.*2.56/5.)-4)
        i += 1
        if i == max_lines: # break if maximum samples is reached
            break
    file.close()
    return (l)

def movingAverageFilter (data, window):
    filtered = []
    average_start = window+1
    for i in range(len(data)):
        if i > average_start:
            average = filtered[-1]+(data[i]-data[i-window])/window
            filtered.append(average)
        else:
            filtered.append(data[i])
    return (filtered)

## RUN SIMULATION ############################################################
print 'Simulation of mine detection experiment 2012-04-13\n'
print 'Press CTRL-C to cancel\n'

# import GPS log
print 'Importing GPS data...'
(gpsLog, gpsNoFixCnt, gpsAboveHDOPCnt) = importGPSLog(GPSLogFile, HDOPThreshold, GPSLines)
print 'Total samples:',(len(gpsLog) + gpsNoFixCnt + gpsAboveHDOPCnt)
print 'Accepted samples:',(len(gpsLog))
print 'Samples with no satellite fix:',gpsNoFixCnt
print 'Samples above HDOP threshold:',gpsAboveHDOPCnt
(minE, maxE, minN, maxN) = gpsBoundaries(gpsLog)

# define simulation time based on GPS log
simOffset = gpsLog[0][GPS_TIME]
simLen = gpsLog[-1][GPS_TIME]-simOffset
simSteps = simLen/simStepInt
print ('\nSimulation step: %.2fs' % simStepInt)
print ('Simulation length %.2fs (%.0f steps)' % (simLen, simSteps))

# import UPEX 740M log
print 'Importing UPEX 740M data...'
upexLog = importUpex740Log (UPEXLogFile, UPEXLines)
print 'Total samples:',len(upexLog)
upexLogT = zip (*upexLog)

upexFiltered = movingAverageFilter (upexLogT[UPEX_ADC], 3)
#upexFiltered = upexLogT[UPEX_ADC]
#for i in range(len(upexLog)):
#    upexLog[i][UPEX_ADC] = upexFiltered[i] # replace measured with filtered values

# setup MathPlotLib plots
ion() # turn interaction mode on
plt.figure(num=1, figsize=(8, 8), dpi=80, facecolor='w', edgecolor='w')
targetPlt = plot(gttT[0], gttT[1],'bx')
plt.scatter(gttT[0],gttT[1],marker='x',c='b',s=35, lw = 1.5)
#title ('Robot Track')
#xlabel('Easting [m]')
#ylabel('Northing [m]')
axis('equal')
grid (False)
ax = gca()
ax.set_frame_on(False)
ax.axes.get_xaxis().set_visible(False)
ax.axes.get_yaxis().set_visible(False)

#plt.figure(2)
#title ('UPEX 740M')
#xlabel('measurements')
#ylabel('Voltage [V]')
#xlim([1,400])
#ylim([-3,4])
#upexplt, = plot([],'r')
#draw()

plot_track_cnt = 0
plot_targets_cnt = 0

# initialize simulation state variables
print "Run simulation..."
simTime = 0.
gpsSim = []
wadsSim = []
gpsIndex = 0
gpsFiltered = 0
gpsFix = 0
upexSim = []
upexIndex = 0
upexAveraging = []
upexFiltered = 0
robot_e = 0.0
robot_n = 0.0
wads_e = 0.0
wads_n = 0.0
wads_yaw = 0.0
wads_yaw_time = 0.0

# start the simulation loop
for step in range ((int(simSteps)+1)):
    # print ('Step %d time %.2f log time %.2f' % (step+1, simTime, (simTime+simOffset)))
    logTime = simTime + simOffset

    # retrieve new GPS measurements
    while gpsLog[gpsIndex][GPS_TIME] <= logTime:
        trkPt = gpsLog[gpsIndex]
        e = trkPt[GPS_E]
        n = trkPt[GPS_N]
        if gpsIndex==0 or sqrt((e-gpsSim[-1][GPS_E])**2+(n-gpsSim[-1][GPS_N])**2) >= TrackMinDist:
			gpsSim.append (trkPt)
        else:
            gpsFiltered += 1
        gpsIndex += 1

    # retrieve new measurements from the UPEX 740M metal detector
    while upexIndex < len(upexLog) and upexLog[upexIndex][UPEX_TIME] <= logTime:
        robot_e = gpsSim[-1][GPS_E] #get latest known geometric center coordinate
        robot_n = gpsSim[-1][GPS_N] 

        wads_yaw_err = (fabs(angle_diff (wads_yaw,  gpsSim[-1][GPS_YAW])) > pi/8.0) \
            and gpsSim[-1][GPS_TIME] - wads_yaw_time < 0.3
        
        if wads_yaw_err:
			print gpsSim[-1][GPS_TIME], fabs(angle_diff (wads_yaw,  gpsSim[-1][GPS_YAW]))*180/pi,wads_yaw*180/pi, gpsSim[-1][GPS_YAW]*180/pi
 
        if wads_yaw_err == False:
            wads_yaw= gpsSim[-1][GPS_YAW] # get robot course
            wads_yaw_time  = gpsSim[-1][GPS_TIME]
            wads_offset = vec2d_rot ([wads_cdist, 0],wads_yaw)
            wads_e = robot_e + wads_offset[0]
            wads_n = robot_n + wads_offset[1]
            wadsSim.append ([wads_e, wads_n])
        
        if wads_yaw_err == False and ptInPoly (wads_e,wads_n,poly): # only consider data obtained inside the site polygon
            upexAveraging.append(upexLog[upexIndex])
            if len(upexSim) == 0 or sqrt((wads_e-upexSim[-1][UPEX_E])**2+(wads_n-upexSim[-1][UPEX_N])**2) >= UPEXSampleDist: #if time to average and save
                upex = []
                upex.append(upexAveraging[-1][UPEX_TIME]) # add time of last sample
                avgSum = 0
                avgN = len(upexAveraging)
                for i in range(avgN):
                    adc = upexAveraging[i][UPEX_ADC]
                    if adc < 0:
                        adc = 0
                    avgSum += adc
                #upex.append(float(avgSum)/avgN)
                upex.append (upexLog[upexIndex][UPEX_ADC])                
                upex.append(wads_n)
                upex.append(wads_e)
                upexSim.append(upex)
                upexAveraging = []
            else:
                upexFiltered += 1
        upexIndex += 1

    # update plots
    if (step) % simPlotUpdate == 0:
        plt.figure(1) # navigation map
        clf()
        xlabel('[m]')
        ylabel('[m]')
        axis('equal')
        grid (True)
        xlim([minE,maxE])
        ylim([minN,maxN])
        gpsSimT = zip (*gpsSim)
        gps_plt = plot(gpsSimT[GPS_E],gpsSimT[GPS_N],'r')
        polyPlt = plot(polyT[0], polyT[1],'g')
        #robotplt = plot(robot_e, robot_n,'rx')
        plt.scatter(gttT[0],gttT[1],marker='x',c='r',s=35, lw = 1.5)

        wplot = []
        for i in xrange(len(wframe)):
			c = vec2d_rot (wframe[i],wads_yaw+pi/2.0)
			wplot.append([c[0]+wads_e,c[1]+wads_n])
        wplotT = zip(*wplot)
        #wadsplt = plot(wplotT[0], wplotT[1],'b')
        plot_track_cnt += 1
        plt.savefig('plot_track/img%05d.jpg' % plot_track_cnt)


      #  if len(upexSim) > 0:
      #      plt.figure(2) # UPEX 740M measurements
      #      upexSimT = zip (*upexSim)
      #      upexplt.set_ydata(upexSimT[UPEX_ADC][-400:-1])
      #      upexplt.set_xdata(range(len(upexSimT[UPEX_ADC][-400:-1])))

        plt.figure(num=3, figsize=(8, 8), dpi=80, facecolor='w', edgecolor='w')
        grid (False)
        plt.clf() # clear figure
        ax = gca()
        #ax.set_frame_on(False)
        #ax.axes.get_xaxis().set_visible(False)
        #ax.axes.get_yaxis().set_visible(False)
#       title ('Targets')
        xlabel('Easting [m]')
        ylabel('Northing [m]')
        xlim([minE,maxE])
        ylim([minN,maxN])
        axis('equal')
        grid (True)
        polyPlt = plot(polyT[0], polyT[1],'g')
        #targetPlt = plot(gttT[0], gttT[1],'bx')
        if len(upexSim) > 0 and step > 4000:
            # extract columns
            upexSimT = zip (*upexSim)
            x = list(upexSimT[UPEX_E])
            y = list(upexSimT[UPEX_N])
            z = list(upexSimT[UPEX_ADC])

            # define grid.
            xi = np.linspace(minE,maxE,300)
            yi = np.linspace(minN,maxN,300)
            zi = griddata((x, y), z, (xi[None,:], yi[:,None]), method='cubic')
            # nearest eturn the value at the data point closest to the point of interpolation. See NearestNDInterpolator for more details.
            # linear esselate the input point set to n-dimensional simplices, and interpolate linearly on each simplex. See LinearNDInterpolator for more details.
            # cubic return the value determined from a piecewise cubic, continuously differentiable (C1), and approximately curvature-minimizing polynomial surface. See CloughTocher2DInterpolator for more details.

            # contour the gridded data, plotting dots at the randomly spaced data points.
            CS = plt.contour(xi,yi,zi,15,linewidths=0.5,colors=('0.5','0.4','0.3'))
#            CS = plt.contour(xi,yi,zi,15,linewidths=0.5,colors='k')
            CS = plt.contourf(xi,yi,zi,15,cmap=plt.cm.YlOrRd)
#            CS = plt.contour(xi,yi,zi,15,cmap=plt.cm.jet)
#            plt.colorbar() # draw colorbar
            plt.scatter(gttT[0],gttT[1],marker='x',c='b',s=35, lw = 1.5)

        wadsSimT = zip (*wadsSim)
        wadstrack_plt = plot(wadsSimT[0],wadsSimT[1],'r')
        wadsplt = plot(wplotT[0], wplotT[1],'b')
        plot_targets_cnt += 1
        plt.savefig('plot_targets/img%05d.jpg' % plot_targets_cnt)

	draw() # redraw plots
    
	#time.sleep (0.01)

    # exit import if CTRL-C pressed
    if Ctrl_C:
        break

    simTime += simStepInt

#plt.figure(1)
#plt.savefig('plot_track.png')
#plt.figure(3)
#plt.savefig('plot_targets.png')


plt.figure(num=3, figsize=(8, 8), dpi=80, facecolor='w', edgecolor='w')
grid (False)
plt.clf() # clear figure
ax = gca()
ax.set_frame_on(False)
ax.axes.get_xaxis().set_visible(False)
ax.axes.get_yaxis().set_visible(False)
#       title ('Targets')
#xlabel('Easting [m]')
#ylabel('Northing [m]')
mapPlotBnd = 0.7
GPS_E = 0
GPS_N = 1
(minE, maxE, minN, maxN) = gpsBoundaries(poly)
xlim([minE,maxE])
ylim([minN,maxN])
axis('equal')
#grid (True)
polyPlt = plot(polyT[0], polyT[1],'g')
#targetPlt = plot(gttT[0], gttT[1],'bx')

wadsSimT = zip (*wadsSim)
wadstrack_plt = plot(wadsSimT[0],wadsSimT[1],'r')
#wadsplt = plot(wplotT[0], wplotT[1],'b')

if True: # len(upexSim) > 0 and step > 4000:
    # extract columns
    upexSimT = zip (*upexSim)
    x = list(upexSimT[UPEX_E])
    y = list(upexSimT[UPEX_N])
    z = list(upexSimT[UPEX_ADC])

    # define grid.
    xi = np.linspace(minE,maxE,1200)
    yi = np.linspace(minN,maxN,1200)
    zi = griddata((x, y), z, (xi[None,:], yi[:,None]), method='cubic')

    # contour the gridded data, plotting dots at the randomly spaced data points.
    #CS = plt.contour(xi,yi,zi,15,linewidths=0.5,colors=('0.5','0.4','0.3'))
    #CS = plt.contourf(xi,yi,zi,15,cmap=plt.cm.YlOrRd)
#            plt.colorbar() # draw colorbar
    plt.scatter(gttT[0],gttT[1],marker='x',c='b',s=35, lw = 1.5)
    draw() # redraw plots
    plt.savefig ('plot_targets.png')



plt.figure(num=5, figsize=(8, 8), dpi=80, facecolor='w', edgecolor='w')

plt.hexbin(x,y, gridsize=100, cmap=plt.cm.YlOrRd_r)
plt.axis([minE, maxE, minN, maxN])


print 'Simulation completed'
print 'GPS positions filtered:',gpsFiltered
print 'UPEX 740M measurements filtered:',upexFiltered

if Ctrl_C == 0:
	raw_input() # wait for enter keypress 

##############################################################################
