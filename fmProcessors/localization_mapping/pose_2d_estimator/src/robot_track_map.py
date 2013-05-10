#!/usr/bin/env python
#/****************************************************************************
# Pose 2d Estimator: Robot Track Map
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
#****************************************************************************/
# imports
import matplotlib.pyplot as plt
from pylab import ion, plot, axis, grid, title, xlabel, ylabel, draw
from math import sqrt

class track_map():
	def __init__(self, enable_pose, enable_gnss, enable_odometry, map_title, map_window_size, easting_offset, northing_offset):
        # Get parameters
		self.odometry_enabled = enable_odometry
		self.gnss_enabled = enable_gnss
		self.pose_enabled = enable_pose
		self.trkpt_threshold = 0.1 # [m]
        
		# Initialize map
		self.offset_e = easting_offset
		self.offset_n = northing_offset
		self.odo = []
		self.gnss = []
		self.pose = []

		ion() # turn interaction mode on
		if self.gnss_enabled or self.pose_enabled:
			self.fig1 = plt.figure(num=1, figsize=(map_window_size, \
				map_window_size), dpi=80, facecolor='w', edgecolor='w')
			if self.gnss_enabled and self.pose_enabled:
				title (map_title + ' - EKF and GNSS')
			elif self.gnss_enabled:
				title (map_title + ' - GNSS')
			elif self.pose_enabled:
				title (map_title + ' - EKF')
			xlabel('Easting [m]')
			ylabel('Northing [m]')
			axis('equal')
			grid (True)

		if self.odometry_enabled:
			self.fig2 = plt.figure(num=2, figsize=(map_window_size, \
				map_window_size), dpi=80, facecolor='w', edgecolor='w')
			title (map_title + ' - Odometry')
			xlabel('[m]')
			ylabel('[m]')
			axis('equal')
			grid (True)

	def append_pose_position (self, easting, northing):
		x = easting + self.offset_e
		y = northing + self.offset_n
		if self.pose == [] or sqrt((x-self.pose[-1][0])**2 + (y-self.pose[-1][1])**2) > self.trkpt_threshold:
			self.pose.append([x, y])

	def append_gnss_position (self, easting, northing):
		x = easting + self.offset_e
		y = northing + self.offset_n
		if self.gnss == [] or sqrt((x-self.gnss[-1][0])**2 + (y-self.gnss[-1][1])**2) > self.trkpt_threshold:
			self.gnss.append([x, y])

	def append_odometry_position (self, x, y):
		if self.odo == [] or sqrt((x-self.odo[-1][0])**2 + (y-self.odo[-1][1])**2) > self.trkpt_threshold:
			self.odo.append([x, y])

	def update(self):
		if self.gnss_enabled and self.gnss != []:
			plt.figure(1)
			gnssT = zip(*self.gnss)		
			gnss_plt = plot(gnssT[0],gnssT[1],'black')
		if self.pose_enabled and self.pose != []:
			plt.figure(1)
			poseT = zip(*self.pose)		
			pose_plt = plot(poseT[0],poseT[1],'r')
		if self.odometry_enabled and self.odo != []:
			plt.figure(2)
			odoT = zip(*self.odo)		
			odo_plt = plot(odoT[0],odoT[1],'b')
		if self.gnss_enabled or self.pose_enabled or self.odometry_enabled:
			draw()

	def save(self, file_name):
		if self.gnss_enabled or self.pose_enabled or self.odometry_enabled:
			self.fig1.savefig (file_name)

	def set_trackpoint_threshold (self, threshold):
		self.trkpt_threshold = threshold 

