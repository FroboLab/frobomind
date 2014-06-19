#!/usr/bin/env python
#*****************************************************************************
# Pose 2D Estimator - Simulation import classes
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
Revision
2013-04-23 KJ First version
"""

# imports
import csv

class odometry_data():
	def __init__(self, filename, skip_lines, max_lines):
		self.i = 0
		print 'Importing odometry data'
		file = open(filename, 'rb')
		file_content = csv.reader(file, delimiter=',')
	 	self.data = []
		i = 0
		for time, x, y, yaw, speed in file_content:
			if i > skip_lines:
				self.data.append([float(time), float(x), float(y), float(yaw), float(speed)])
			i += 1
			if max_lines > 0 and i == max_lines:
				break
		file.close()
		self.length = len(self.data)
		print '\tTotal samples: %d' % (self.length) 

	def get_latest(self, time):
		new_data = 0
		while self.i < self.length and self.data[self.i][0] <= time:
			# print ('  Odometry time: %f' % (self.data[self.i][0]))
			self.i += 1
			new_data += 1
		return (new_data, self.data[self.i-1])

class imu_data():
	def __init__(self, filename, skip_lines, max_lines):
		self.i = 0
		print 'Importing IMU data'
		file = open(filename, 'rb')
		file_content = csv.reader(file, delimiter=',')
	 	self.data = []
		i = 0
		for time, ang_vel_z, orient_yaw in file_content:
			if i > skip_lines:
				self.data.append([float(time), float(ang_vel_z), float(orient_yaw)])
			i += 1
			if max_lines > 0 and i == max_lines:
				break
		file.close()
		self.length = len(self.data)
		print '\tTotal samples: %d' % (self.length) 

	def get_latest(self, time):
		new_data = 0
		while self.i < self.length and self.data[self.i][0] <= time:
			self.i += 1
			new_data += 1
		return (new_data, self.data[self.i-1])

class gnss_data:
	def __init__(self, filename, skip_lines, max_lines):
		self.i = 0
		print 'Importing GPS data'
		file = open(filename, 'rb')
		file_content = csv.reader(file, delimiter=',')
	 	self.data = []
		i = 0
		origo_e = 0
		origo_n = 0
		for time, lat, lon, easting, northing, fix, sat, hdop in file_content:
			if i > skip_lines:
				self.data.append([float(time), float(lat), float(lon), \
					float(easting), float(northing), int(fix), int(sat), float(hdop)])
			i += 1
			if max_lines > 0 and i == max_lines:
				break
		file.close()
		self.length = len(self.data)
		print '\tTotal samples: %d' % (self.length) 

	def get_latest(self, time):
		new_data = 0
		while self.i < self.length and self.data[self.i][0] <= time:
			# print ('  GPS time: %f' % (self.data[self.i][0]))
			self.i += 1
			new_data += 1
		if self.i < self.length:
			return (new_data, self.data[self.i-1])
		else:
			return (0, False)

