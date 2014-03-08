#!/usr/bin/env python
#*****************************************************************************
# Waypoint Navigation - Simulation import classes
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
2013-06-20 KJ First version
"""

# imports
import csv

class pose_data():
	def __init__(self, filename, skip_lines, max_lines):
		self.i = 0
		print 'Importing pose data'
		file = open(filename, 'r')
		file_content = csv.reader(file, delimiter=',')
	 	self.data = []
		i = 0
		for time, x, y, yaw, linspd, angspd in file_content:
			if i > skip_lines:
				self.data.append([float(time), float(x), float(y), float(yaw), float(linspd), float(angspd)])
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

class wptnav_data():
	def __init__(self, filename, skip_lines, max_lines):
		self.i = 0
		print 'Importing waypoint navigation status data'
		file = open(filename, 'r')
		file_content = csv.reader(file, delimiter=',')
	 	self.data = []
		i = 0
		for time_stamp,mode,b_e,b_n,a_e,a_n, \
			e,n,dist_b,bearing_b,head_err,dist_ab, \
			t_e,t_n,t_dist,t_bearing,t_head_err, \
			linspd,angspd in file_content:
			if i > skip_lines:
				self.data.append([float(time_stamp), int(mode), float(b_e), float(b_n), float(a_e), float(a_n), \
					float(e), float(n), float(dist_b), float(bearing_b), float(head_err), float(dist_ab), \
					float(t_e), float(t_n), float(t_dist), float(t_bearing), float(t_head_err), \
					float(linspd), float(angspd)])
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

class cmd_vel_data():
	def __init__(self, filename_left, filename_right, skip_lines, max_lines):
		self.i_left = 0
		self.i_right = 0
		print 'Importing set_speed data'
	 	self.data_left = []
	 	self.data_right = []
		file = open(filename_left, 'r')
		file_content = csv.reader(file, delimiter=',')
		i = 0
		for time_stamp,speed in file_content:
			if i > skip_lines:
				self.data_left.append([float(time_stamp), float(speed)])
			i += 1
			if max_lines > 0 and i == max_lines:
				break
		file.close()
		self.length_left = len(self.data_left)
		print '\tTotal samples left: %d' % (self.length_left) 

		file = open(filename_right, 'r')
		file_content = csv.reader(file, delimiter=',')
		i = 0
		for time_stamp,speed in file_content:
			if i > skip_lines:
				self.data_right.append([float(time_stamp), float(speed)])
			i += 1
			if max_lines > 0 and i == max_lines:
				break
		file.close()
		self.length_right = len(self.data_right)
		print '\tTotal samples right: %d' % (self.length_right) 

	def get_latest_left(self, time):
		new_data = 0
		while self.i_left < self.length_left and self.data_left[self.i_left][0] <= time:
			self.i_left += 1
			new_data += 1
		return (new_data, self.data_left[self.i_left-1])

	def get_latest_right(self, time):
		new_data = 0
		while self.i_right < self.length_right and self.data_right[self.i_right][0] <= time:
			self.i_right += 1
			new_data += 1
		return (new_data, self.data_right[self.i_right-1])


