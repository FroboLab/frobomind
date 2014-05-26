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
"""
This file contains a 2D pose mapping class designed for field robots.

The pose follows the ENU coordinate system to allow use of GNSS based
coordinates represented in a Transverse Mercator projection. 

	Pose position vector: [easting, northing]
	Pose orientation vector: [yaw] positive rotation CCW origo at the easting axis 

Revision
2013-04-25 KJ First version
"""
# imports
import matplotlib.pyplot as plt
from pylab import ion, plot, axis, grid, title, xlabel, ylabel, draw
from math import pi, sqrt

class track_map():
	def __init__(self, plot_pose, plot_gnss, plot_odometry, plot_yaw, map_title, map_window_size, easting_offset, northing_offset):
		self.rad_to_deg = 180.0/pi
        # Get parameters
		self.plot_pose = plot_pose
		self.plot_gnss = plot_gnss
		self.plot_odometry = plot_odometry
		self.plot_yaw = plot_yaw
		self.trkpt_threshold = 0.1 # [m]
        
		# Initialize map
		self.offset_e = easting_offset
		self.offset_n = northing_offset
		self.odo = []
		self.gnss = []
		self.pose_pos = []
		self.odo_yaw = []
		self.gnss_yaw = []
		self.ahrs_yaw = []
		self.pose_yaw = []

		ion() # turn interaction mode on
		if self.plot_gnss or self.plot_pose:
			self.fig1 = plt.figure(num=1, figsize=(map_window_size, \
				map_window_size), dpi=80, facecolor='w', edgecolor='w')
			if self.plot_gnss and self.plot_pose:
				title (map_title + ' - Pose and GNSS')
			elif self.plot_gnss:
				title (map_title + ' - GNSS')
			elif self.plot_pose:
				title (map_title + ' - Pose')
			xlabel('Easting [m]')
			ylabel('Northing [m]')
			axis('equal')
			grid (True)

			# this is a temporary hack to show the SDU test field
			"""sdu_test_field = [[588787.7447,6137275.3330], [588795.7513,6137275.1412], [588795.9340,6137283.1338], [588787.9458,6137283.3454], [588788.1267,6137291.3197], [588796.1145,6137291.1133]]
			sdu_test_field.pop (-1)
			sdu_test_field.pop (-1)
			for i in xrange(len(sdu_test_field)):
				sdu_test_field[i][0] += self.offset_e
				sdu_test_field[i][1] += self.offset_n
			sdu_test_fieldT = zip(*sdu_test_field)
			plot (sdu_test_fieldT[0], sdu_test_fieldT[1], 'bo')"""

		if self.plot_odometry:
			self.fig2 = plt.figure(num=2, figsize=(map_window_size, \
				map_window_size), dpi=80, facecolor='w', edgecolor='w')
			title (map_title + ' - Odometry')
			xlabel('[m]')
			ylabel('[m]')
			axis('equal')
			grid (True)

		if self.plot_yaw:
			self.fig3 = plt.figure(num=3, figsize=(map_window_size, \
				map_window_size), dpi=80, facecolor='w', edgecolor='w')
			title (map_title + ' - Orientation (yaw)')
			xlabel('Time')
			ylabel('Orientation [deg]')
			grid (True)

	def append_pose_position (self, easting, northing):
		x = easting + self.offset_e
		y = northing + self.offset_n
		if self.pose_pos == [] or sqrt((x-self.pose_pos[-1][0])**2 + (y-self.pose_pos[-1][1])**2) > self.trkpt_threshold:
			self.pose_pos.append([x, y])

	def append_gnss_position (self, easting, northing):
		x = easting + self.offset_e
		y = northing + self.offset_n
		if self.gnss == [] or sqrt((x-self.gnss[-1][0])**2 + (y-self.gnss[-1][1])**2) > self.trkpt_threshold:
			self.gnss.append([x, y])

	def append_odometry_position (self, x, y):
		if self.odo == [] or sqrt((x-self.odo[-1][0])**2 + (y-self.odo[-1][1])**2) > self.trkpt_threshold:
			self.odo.append([x, y])

	def append_pose_yaw (self, yaw):
		self.pose_yaw.append(yaw*self.rad_to_deg)

	def append_gnss_yaw (self, yaw):
		self.gnss_yaw.append(yaw*self.rad_to_deg)

	def append_ahrs_yaw (self, yaw):
		self.ahrs_yaw.append(yaw*self.rad_to_deg)

	def append_odo_yaw (self, yaw):
		self.odo_yaw.append(yaw*self.rad_to_deg)

	def update(self):
		if self.plot_gnss and self.gnss != []:
			plt.figure(1)
			gnssT = zip(*self.gnss)		
			gnss_plt = plot(gnssT[0],gnssT[1],'black')
		if self.plot_pose and self.pose_pos != []:
			plt.figure(1)
			poseT = zip(*self.pose_pos)		
			pose_plt = plot(poseT[0],poseT[1],'r')
		if self.plot_odometry and self.odo != []:
			plt.figure(2)
			odoT = zip(*self.odo)		
			odo_plt = plot(odoT[0],odoT[1],'b')
		if self.plot_yaw:
			if  self.odo_yaw != []:
				plt.figure(3)
				odo_yaw_plt = plot(self.odo_yaw,'b')
			if  self.ahrs_yaw != []:
				plt.figure(3)
				ahrs_yaw_plt = plot(self.ahrs_yaw,'g')
			if  self.gnss_yaw != []:
				plt.figure(3)
				gnss_yaw_plt = plot(self.gnss_yaw, 'black')
			if  self.pose_yaw != []:
				plt.figure(3)
				pose_yaw_plt = plot(self.pose_yaw,'r')
		if self.plot_gnss or self.plot_pose or self.plot_odometry or self.plot_yaw:
			draw()

	def save(self, file_name):
		if self.plot_gnss or self.plot_pose:
			if self.plot_gnss and self.plot_pose:
				self.fig1.savefig (file_name+'-gnss-pose.png')
			elif self.plot_gnss:
				self.fig1.savefig (file_name+'-gnss.png')
			else:
				self.fig1.savefig (file_name+'-pose.png')
		if self.plot_odometry:
			self.fig2.savefig (file_name+'-odometry.png')
		if self.plot_yaw:
			self.fig3.savefig (file_name+'-yaw.png')

	def set_trackpoint_threshold (self, threshold):
		self.trkpt_threshold = threshold 

