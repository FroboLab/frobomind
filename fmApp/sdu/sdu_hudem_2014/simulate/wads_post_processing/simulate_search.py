#!/usr/bin/env python

## PARAMETERS ################################################################
PoseLogFile = 'sim_pose.txt'
PoseLogSkipLines = 0
PoseLogMaxLines = 8000000 # maximum number of lines to read from the log file
#GPSLogFile = 'sim_gnss.txt'
#GPSLogSkipLines = 0
#GPSLogMaxLines = 800000 # maximum number of lines to read from the log file
HDOPThreshold = 10. # GPS fix above this HDOP value will be ignored
OrigoE = 588772 # subtract from all Easting coordinates
OrigoN = 6137285 # subtract from all Northing coordinates
TrackMinDist = 0.02 # [m] minimum distance between consequtive track points
#HeadingMinDist = 0.25 # [m] minimum distance between track points when calculating heading

wads_cdist = 1.66 # distance from GPS antenna to center of implement

UPEXLogFile = 'sim_wads.txt'
UPEXLines = 50000000
UPEXSampleDist = 0.02

figure_size = 8
grid_size = 2000

simStepInt = 0.02 # simulation step interval [s]

simPlotUpdate = 2000000 # number of steps between updating the plots
mapPlotBnd = 3.0 # border around map plots [m] 

SaveMovieImgs = False


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

# define imported pose data columns
POSE_TIME = 0
POSE_E = 1
POSE_N = 2
POSE_YAW = 3

# define imported GPS data columns
GPS_TIME = 0
GPS_E = 1
GPS_N = 2
GPS_FIX = 3

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

def mapBoundaries(e, n):
	minE = float(1e1000)
	minN = float(1e1000)
	maxE = -float(1e1000)
	maxN = -float(1e1000)
	for i in range(len(e)):
		if minE > e[i]:
			minE = e[i]
		elif maxE < e[i]:
			maxE = e[i]
		if minN > n[i]:
			minN = n[i]
		elif maxN < n[i]:
			maxN = n[i]
	return (minE-mapPlotBnd, maxE+mapPlotBnd, minN-mapPlotBnd, maxN+mapPlotBnd)

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
(minE, maxE, minN, maxN) = mapBoundaries(polyT[0], polyT[1])

# ground truth target list
gtt = [[588778.9326,6137291.6141],[588773.3388,6137293.7092],[588776.2196,6137299.5903],[588775.1004,6137303.9440],[588778.7879,6137303.6453],[588779.7928,6137296.2574]]

for i in range(len(gtt)):
	gtt[i][0] -= OrigoE
	gtt[i][1] -= OrigoN
gttT = zip (*gtt)

minE, maxE, minN, maxN
## Pose #######################################################################
def importPoseLog (filename, skip_lines, max_lines):
	file = open(filename, 'rb')
	data = csv.reader(file, delimiter=',')
	cnt = 0
	i=0
	l = []
	for time, e, n, yaw in data:
		cnt += 1	
		if cnt > skip_lines:
			l.append([])
			l[i].append (float(time))
			l[i].append (float(e)-OrigoE)
			l[i].append (float(n)-OrigoN)
			l[i].append (float(yaw))
			i += 1
		if i == max_lines: # break if maximum samples is reached
			break
	file.close()
	return (l)

## GPS #######################################################################
def importGPSLog (filename, hdop_threshold, skip_lines, max_lines):
	file = open(filename, 'rb')
	data = csv.reader(file, delimiter=',')
	cnt = 0
	i=0
	l = []
	samples_nofix = 0
	samples_above_hdop = 0
	for time, utc, lat, lon, utme, utmn, fix, sat, hdop in data:
		cnt += 1	
		if cnt > skip_lines: #int(fix) > 0:
			if True: #float(hdop) <= hdop_threshold:
				l.append([])
				l[i].append (float(time))
				l[i].append (float(utme)-OrigoE)
				l[i].append (float(utmn)-OrigoN)
				l[i].append (float(fix))
				i += 1
			else:
				samples_above_hdop += 1
		else:
			samples_nofix += 1
		if i == max_lines: # break if maximum samples is reached
			break

	file.close()
	return (l,samples_nofix, samples_above_hdop)


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
		#l[i].append (int(adc)*8./(1023.*2.56/5.)-4)
		l[i].append (int(adc))
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

##############################################################################
def plot_search_map (search_log):
	figure(num=6, figsize=(figure_size,figure_size), dpi=80, facecolor='w', edgecolor='w')
	plt.clf() # clear figure
	plt.title ('Search map')
	xlabel('Easting [m]')
	ylabel('Northing [m]')
	axis('equal')
	grid (True)
	xlim([minE,maxE])
	ylim([minN,maxN])
		# extract columns
	slT = zip (*search_log)
	x = list(slT[UPEX_E])
	y = list(slT[UPEX_N])
	z = list(slT[UPEX_ADC])

	# define grid.
	xi = np.linspace(minE,maxE,grid_size)
	yi = np.linspace(minN,maxN,grid_size)
	zi = griddata((x, y), z, (xi[None,:], yi[:,None]), method='linear')
	# nearest eturn the value at the data point closest to the point of interpolation. See NearestNDInterpolator for more details.
	# linear esselate the input point set to n-dimensional simplices, and interpolate linearly on each simplex. See LinearNDInterpolator for more details.
	# cubic return the value determined from a piecewise cubic, continuously differentiable (C1), and approximately curvature-minimizing polynomial surface. See CloughTocher2DInterpolator for more details.


	# plot kanten
	#CS = plt.contour(xi,yi,zi,10,linewidths=0.5,colors=('0.5','0.4','0.3'))

#			CS = plt.contour(xi,yi,zi,15,linewidths=0.5,colors='k')
	CS = plt.contourf(xi,yi,zi,100, cmap=plt.cm.YlOrRd)
	#plt.colorbar() # draw colorbar
	polyPlt = plot(polyT[0], polyT[1],'g') # draw ploygon
	plt.scatter(gttT[0],gttT[1],marker='x',c='b',s=35, lw = 1.5) # draw target's known place

##############################################################################
def plot_track_wads (poly, wads_log):
	plt.figure(num=7, figsize=(figure_size,figure_size), dpi=80, facecolor='w', edgecolor='w')
	plt.clf() # clear figure
	plt.title ('WADS track')
	xlabel('Easting [m]')
	ylabel('Northing [m]')
	axis('equal')
	grid (True)
	plt.xlim([minE,maxE])
	plt.ylim([minN,maxN])

	#ax = gca()
	#ax.set_frame_on(False)
	#ax.axes.get_xaxis().set_visible(False)
	#ax.axes.get_yaxis().set_visible(False)

	#targetPlt = plot(gttT[0], gttT[1],'bx')
	wadsSimT = zip (*wads_log)
	wadstrack_plt = plot(wadsSimT[0],wadsSimT[1],'r')
	#wadsplt = plot(wplotT[0], wplotT[1],'b')
	polyPlt = plot(polyT[0], polyT[1],'g') # draw ploygon
	plt.scatter(gttT[0],gttT[1],marker='x',c='b',s=35, lw = 1.5) # draw target's known place


## RUN SIMULATION ############################################################
print 'Simulation of mine detection experiment 2012-04-13\n'
print 'Press CTRL-C to cancel\n'

# import pose log
print 'Importing Pose data...'
(poseLog) = importPoseLog(PoseLogFile, PoseLogSkipLines, PoseLogMaxLines)
print 'Total samples:',(len(poseLog))
pT = zip(*poseLog)
(minE, maxE, minN, maxN) = mapBoundaries(pT[1], pT[2])

# import GPS log
#print 'Importing GPS data...'
#(gpsLog, gpsNoFixCnt, gpsAboveHDOPCnt) = importGPSLog(GPSLogFile, HDOPThreshold, GPSLogSkipLines, GPSLogMaxLines)
#print 'Total samples:',(len(gpsLog) + gpsNoFixCnt + gpsAboveHDOPCnt)
#print 'Accepted samples:',(len(gpsLog))
#print 'Samples with no satellite fix:',gpsNoFixCnt
#print 'Samples above HDOP threshold:',gpsAboveHDOPCnt

# define simulation time based on GPS log
simOffset = poseLog[0][GPS_TIME]
simLen = poseLog[-1][GPS_TIME]-simOffset
simSteps = simLen/simStepInt
print ('\nSimulation step: %.2fs' % simStepInt)
print ('Simulation length %.2fs (%.0f steps)' % (simLen, simSteps))

# import UPEX 740M log
print 'Importing UPEX 740M data...'
upexLog = importUpex740Log (UPEXLogFile, UPEXLines)

minupex = 5000
maxupex = 0
for i in xrange(len(upexLog)):
	if upexLog[i][1] < minupex:
		minupex = upexLog[i][1]
	if upexLog[i][1] > maxupex:
		maxupex = upexLog[i][1]
print minupex, maxupex

for i in xrange(len(upexLog)):
	upexLog[i][1] -= minupex

print 'Total samples:',len(upexLog)
upexLogT = zip (*upexLog)

upexFiltered = movingAverageFilter (upexLogT[UPEX_ADC], 2)
#upexFiltered = upexLogT[UPEX_ADC]
#for i in range(len(upexLog)):
#	upexLog[i][UPEX_ADC] = upexFiltered[i] # replace measured with filtered values

# setup MathPlotLib plots
ion() # turn interaction mode on

plot_track_cnt = 0
plot_targets_cnt = 0

# initialize simulation state variables
print "Run simulation..."
simTime = 0.
poseSim = []
gpsSim = []
wadsSim = []
poseIndex = 0
poseFiltered = 0
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
	logTime = simTime + simOffset

	# retrieve new pose
	while poseLog[poseIndex][GPS_TIME] <= logTime:
		trkPt = poseLog[poseIndex]
		e = trkPt[GPS_E]
		n = trkPt[GPS_N]
		if poseIndex==0 or sqrt((e-poseSim[-1][GPS_E])**2+(n-poseSim[-1][GPS_N])**2) >= TrackMinDist:
			poseSim.append (trkPt)
		else:
			poseFiltered += 1
		poseIndex += 1

	# retrieve new GPS
	#while gpsIndex < len(gpsLog) and gpsLog[gpsIndex][GPS_TIME] <= logTime:
	#	trkPt = gpsLog[gpsIndex]
	#	e = trkPt[GPS_E]
	#	n = trkPt[GPS_N]
	#	if gpsIndex==0 or sqrt((e-gpsSim[-1][GPS_E])**2+(n-gpsSim[-1][GPS_N])**2) >= TrackMinDist:
	#		gpsSim.append (trkPt)
	#	else:
	#		gpsFiltered += 1
	#	gpsIndex += 1

	# retrieve new measurements from the UPEX 740M metal detector
	while upexIndex < len(upexLog) and upexLog[upexIndex][UPEX_TIME] <= logTime:
		robot_e = poseSim[-1][POSE_E] #get latest known geometric center coordinate
		robot_n = poseSim[-1][POSE_N] 

		wads_yaw_err = (fabs(angle_diff (wads_yaw,  poseSim[-1][POSE_YAW])) > pi/8.0) \
			and poseSim[-1][POSE_TIME] - wads_yaw_time < 0.3
		
		#gps_fix_err = len(gpsLog) > 0 and gpsLog[-1][GPS_FIX] != 4

		if wads_yaw_err:
			print poseSim[-1][POSE_TIME], fabs(angle_diff (wads_yaw,  poseSim[-1][POSE_YAW]))*180/pi,wads_yaw*180/pi, poseSim[-1][POSE_YAW]*180/pi

		if wads_yaw_err == False: # and gps_fix_err == False:
			wads_yaw= poseSim[-1][POSE_YAW] # get robot course
			wads_yaw_time  = poseSim[-1][POSE_TIME]
			wads_offset = vec2d_rot ([wads_cdist, 0],wads_yaw)
			wads_e = robot_e + wads_offset[0]
			wads_n = robot_n + wads_offset[1]
			wadsSim.append ([wads_e, wads_n])
		
		if wads_yaw_err == False and ptInPoly (wads_e,wads_n,poly): # only consider data obtained inside the site polygon
			upexAveraging.append(upexLog[upexIndex])
			if True: # len(upexSim) == 0 or sqrt((wads_e-upexSim[-1][UPEX_E])**2+(wads_n-upexSim[-1][UPEX_N])**2) >= UPEXSampleDist: #if time to average and save
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
	if False: #step % simPlotUpdate == 0:
		plt.figure(num=1, figsize=(figure_size,figure_size), dpi=80, facecolor='w', edgecolor='w')
		clf()
		xlabel('[m]')
		ylabel('[m]')
		axis('equal')
		grid (True)
		xlim([minE,maxE])
		ylim([minN,maxN])
		poseSimT = zip (*poseSim)
		pose_plt = plot(poseSimT[GPS_E],poseSimT[GPS_N],'r')
		polyPlt = plot(polyT[0], polyT[1],'g')
		#robotplt = plot(robot_e, robot_n,'rx')

		wplot = []
		for i in xrange(len(wframe)):
			c = vec2d_rot (wframe[i],wads_yaw+pi/2.0)
			wplot.append([c[0]+wads_e,c[1]+wads_n])
		wplotT = zip(*wplot)

		wadsplt = plot(wplotT[0], wplotT[1],'b')
		
		plt.scatter(gttT[0],gttT[1],marker='x',c='b',s=35, lw = 1.5)
		if SaveMovieImgs == True:
			plot_track_cnt += 1
			plt.savefig('plot_track/img%05d.jpg' % plot_track_cnt)

		plt.figure(num=2, figsize=(figure_size,figure_size), dpi=80, facecolor='w', edgecolor='w')
		clf()
		xlabel('[m]')
		ylabel('[m]')
		axis('equal')
		grid (True)
		xlim([minE,maxE])
		ylim([minN,maxN])
		wadsSimT = zip (*wadsSim)
		#wadstrack_plt = plot(wadsSimT[0],wadsSimT[1],'r')
		polyPlt = plot(polyT[0], polyT[1],'g')
		#robotplt = plot(robot_e, robot_n,'rx')

		wplot = []
		for i in xrange(len(wframe)):
			c = vec2d_rot (wframe[i],wads_yaw+pi/2.0)
			wplot.append([c[0]+wads_e,c[1]+wads_n])
		wplotT = zip(*wplot)
		wadsplt = plot(wplotT[0], wplotT[1],'b')
		plt.scatter(gttT[0],gttT[1],marker='x',c='b',s=35, lw = 1.5)
		if SaveMovieImgs == True:
			plot_track_cnt += 1
			plt.savefig('plot_track/img%05d.jpg' % plot_track_cnt)



	  #  if len(upexSim) > 0:
	  #	  plt.figure(2) # UPEX 740M measurements
	  #	  upexSimT = zip (*upexSim)
	  #	  upexplt.set_ydata(upexSimT[UPEX_ADC][-400:-1])
	  #	  upexplt.set_xdata(range(len(upexSimT[UPEX_ADC][-400:-1])))

		plt.figure(num=3, figsize=(figure_size,figure_size), dpi=80, facecolor='w', edgecolor='w')
		grid (False)
		plt.clf() # clear figure
		ax = gca()
		#ax.set_frame_on(False)
		#ax.axes.get_xaxis().set_visible(False)
		#ax.axes.get_yaxis().set_visible(False)
#	   title ('Targets')
		xlabel('Easting [m]')
		ylabel('Northing [m]')
		xlim([minE,maxE])
		ylim([minN,maxN])
		axis('equal')
		grid (True)
		polyPlt = plot(polyT[0], polyT[1],'g')
		#targetPlt = plot(gttT[0], gttT[1],'bx')
		#if len(upexSim) > 0 and step > 4000:
		#	plot_search_map (upexSim)


		wadsSimT = zip (*wadsSim)
		#wadstrack_plt = plot(wadsSimT[0],wadsSimT[1],'b')
		wadsplt = plot(wplotT[0], wplotT[1],'b')
		if SaveMovieImgs == True:
			plot_targets_cnt += 1
			plt.savefig('plot_targets/img%05d.jpg' % plot_targets_cnt)
		plt.savefig('plot_search_map.png')

		draw() # redraw plots
	
	#time.sleep (0.01)

	# exit import if CTRL-C pressed
	if Ctrl_C:
		break

	simTime += simStepInt


plot_search_map (upexSim)
plt.savefig('plot_search_map.png')

plot_track_wads (poly, wadsSim)
plt.savefig('plot_track_wads.png')

print 'Simulation completed'
#print 'GPS positions filtered:',gpsFiltered
print 'UPEX 740M measurements filtered:',upexFiltered

if Ctrl_C == 0:
	raw_input() # wait for enter keypress 

##############################################################################
