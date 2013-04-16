#!/usr/bin/env python
#*****************************************************************************
# Pose 2D Estimator - robot drive simulation
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
This file contains a Python script to test the Pose 2D Estimator using system
feedback and sensor data from test drives with the Armadillo Pichi field 
robot platform: http://www.frobomind.org/index.php?title=Robot:Armadillo_Pichi

Revision
2013-04-11 KJ First version
"""

# imports
import csv
from utm import utmconv

# parameters
odom_file = 'sim_odometry.txt'
odom_max_lines = 0
gps_file = 'sim_gps.txt'
gps_max_lines = 0
sim_step_int = 0.01 # 100 Hz
relative_coordinates = True

# import odometry
def import_odometry (filename, max_lines):
	print 'Importing odometry data'
	file = open(filename, 'rb')
	file_content = csv.reader(file, delimiter=',')
 	data = []
	i = 0
	for time, x, y, yaw in file_content:
		data.append([time, x, y, yaw])
		i += 1
		if max_lines > 0 and i == max_lines:
			break
	file.close()
	print '\tTotal samples: %d' % (len(data)) 
	return (data)

# import gps
def import_gps (filename, max_lines):
	print 'Importing GPS data'
	file = open(filename, 'rb')
	file_content = csv.reader(file, delimiter=',')
 	data = []
	i = 0
	uc = utmconv()
	orito_e = 0
	origo_n = 0
	for time, lat, lon, fix, sat, hdop in file_content:
		if fix > 0:
			# convert to UTM
			(hemisphere, zone, letter, easting, northing) = uc.geodetic_to_utm (lat,lon)
			# use relative coordinates
			if relative_coordinates:
				if origo_e == 0:
					origo_e = easting
					origo_n = northing
				else:
					easting -= origo_e
					northing -= origo_n
		else:
			easting = 0
			northing = 0
		data.append([time, lat, lon, fix, sat, hdop, easting, northing])
		i += 1
		if max_lines > 0 and i == max_lines:
			break
	file.close()
	print '\tTotal samples: %d' % (len(data)) 
	return (data)

# main
print 'Simulation of robot motion'
print 'Press CTRL-C to cancel'

# import simulation data
odo_sim = import_odometry (odom_file, odom_max_lines)
gps_sim = import_gps (gps_file, gps_max_lines)

# define simulation time based on GPS log
sim_offset = gps_sim[0][0]
sim_len = gps_sim[-1][0] - sim_offset
simSteps = sim_len/sim_step_int
print ('Simulation')
print ('  Step interval: %.2fs' % sim_step_int)
print ('  Total: %.2fs (%.0f steps)' % (sim_len, sim_steps))

