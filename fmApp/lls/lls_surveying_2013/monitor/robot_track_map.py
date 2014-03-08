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
Revision
2013-08-14 KJ First version
2013-09-23 KJ Corrected a bug wen only plotting the pose position
2013-11-22 KJ Added support for drawing robot avatars
"""
# imports
import matplotlib.pyplot as plt
from pylab import ion, plot, axis, grid, title, xlabel, ylabel, draw, clf
from math import pi, sqrt, sin, cos
import csv

class track_map():
	def __init__(self, plot_pose, plot_gnss, plot_odometry, plot_yaw, map_title, map_window_size, easting_offset, northing_offset):
		self.rad_to_deg = 180.0/pi
		self.deg_to_rad = pi/180.0
        # Get parameters
		self.plot_pose = plot_pose
		self.plot_gnss = plot_gnss
		self.plot_odometry = plot_odometry
		self.plot_yaw = plot_yaw
		self.trkpt_threshold = 0.1 # [m]
		self.save_time_lapse_images = False
        
		# Initialize map
		self.offset_e = easting_offset
		self.offset_n = northing_offset
		self.map_window_size = map_window_size
		self.map_title = map_title
		self.map_image_cnt = 0
		self.odo = []
		self.gnss = []
		self.pose_pos = []
		self.odo_yaw = []
		self.gnss_yaw = []
		self.ahrs_yaw = []
		self.pose_yaw = []
		self.wpt_mode = 0
		self.wpt_a = False
		self.wpt_b = False
		self.wpt_target = False
		self.wpt_list = []
		self.wpt_first = True
		#self.robot_avatar = [[-0.4,-0.25],[-0.4,0.25],[0.4,0.0],[-0.4,-0.25]] # Arrow
		self.robot_avatar = [[-0.505,-0.315],[-0.505,0.315],[0.195,0.315],[0.195,-0.315],[-0.505,-0.315],[-0.505,-0.19],[0.195,-0.19],[0.195,0.19],[-0.505,0.19]] # FroboScout


		ion() # turn interaction mode on
		if self.plot_gnss or self.plot_pose:
			self.fig1 = plt.figure(num=1, figsize=(self.map_window_size, \
				self.map_window_size), dpi=80, facecolor='w', edgecolor='w')

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
			xlabel('time')
			ylabel('[deg]')
			grid (True)

	def vec2d_rot (self, v, theta): # return the vector v rotated by theta
		rot_x = v[0]*cos(theta) - v[1]*sin(theta)
		rot_y = v[0]*sin(theta) + v[1]*cos(theta)
		return ([rot_x, rot_y])

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

	def set_wptnav (self, mode, a_easting, a_northing, b_easting, b_northing, target_easting, target_northing):
		self.wpt_mode = mode
		self.wpt_a = [a_easting + self.offset_e, a_northing + self.offset_n]
		self.wpt_b = [b_easting + self.offset_e, b_northing + self.offset_n]
		if self.wpt_first == True:
			self.wpt_first = False
			self.wpt_list.append(self.wpt_a)
		self.wpt_list.append(self.wpt_b)
		self.wpt_target = [target_easting + self.offset_e, target_northing + self.offset_n]

	def update(self):
		if self.plot_gnss or self.plot_pose:
			plt.figure(1)
			clf()
			self.fig1 = plt.figure(num=1, figsize=(self.map_window_size, \
				self.map_window_size), dpi=80, facecolor='w', edgecolor='w')
			title (self.map_title)			
			xlabel('Easting [m]')
			ylabel('Northing [m]')
			axis('equal')
			grid (True)
			track_plot_reset = True	

		if (self.plot_gnss and self.gnss != []):
			#plot (self.test_fieldT[0], self.test_fieldT[1], 'g')
			#plot (self.test_fieldT[0], self.test_fieldT[1], 'go',markersize=7)

			wptlistT = zip(*self.wpt_list)
			wptlist_plt = plot(wptlistT[0],wptlistT[1],'#ff0000')
			plot (self.wpt_a[0], self.wpt_a[1], 'g')
			
			gnssT = zip(*self.gnss)		
			gnss_plt = plot(gnssT[0],gnssT[1],'#000000')

		if self.plot_pose and self.pose_pos != []:
			poseT = zip(*self.pose_pos)		
			pose_plt = plot(poseT[0],poseT[1],'r')
		if self.plot_pose or self.plot_gnss:
 
			if self.pose_yaw != [] and self.pose_pos != []:
				yaw = self.pose_yaw[-1]*self.deg_to_rad
				east = self.pose_pos[-1][0]
				north = self.pose_pos[-1][1]

				# define avatar plot
				avatar_plot = []
				for i in xrange(len(self.robot_avatar)):
					c = self.vec2d_rot (self.robot_avatar[i],yaw)
					avatar_plot.append([c[0]+east,c[1]+north])
				avatar_plotT = zip(*avatar_plot)
				ava_plt = plot(avatar_plotT[0],avatar_plotT[1],'b')

			if self.wpt_mode == 1 or self.wpt_mode == 2:
				if self.wpt_mode > 0 and self.wpt_b != False:
					a_plt = plot(self.wpt_a[0],self.wpt_a[1],'go',markersize=6)
					b_plt = plot(self.wpt_b[0],self.wpt_b[1],'ro',markersize=6)
				if self.wpt_mode == 1:
					if self.pose_pos != [] and (self.pose_pos[-1][0] != 0.0  or self.pose_pos[-1][1] != 0.0):
						pose_plt = plot(self.pose_pos[-1][0],self.pose_pos[-1][1],'bs',markersize=6)
					if self.wpt_target != False:
						target_plt = plot(self.wpt_target[0],self.wpt_target[1],'ro',markersize=5)
				elif self.wpt_mode == 2:
					if self.pose_pos != [] and (self.pose_pos[-1][0] != 0.0  or self.pose_pos[-1][1] != 0.0):
						pose_plt = plot(self.pose_pos[-1][0],self.pose_pos[-1][1],'bo',markersize=6)
					if self.wpt_target != False:
						target_plt = plot(self.wpt_target[0],self.wpt_target[1],'ro',markersize=5)
			elif self.wpt_mode == -1:
				if self.pose_pos != [] and (self.pose_pos[-1][0] != 0.0  or self.pose_pos[-1][1] != 0.0):
					pose_plt = plot(self.pose_pos[-1][0],self.pose_pos[-1][1],'b^',markersize=6)

		if self.save_time_lapse_images == True:
			self.fig1.savefig ('img%05d.jpg' % self.map_image_cnt)
			self.map_image_cnt += 1

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

