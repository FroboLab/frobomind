#!/usr/bin/env python
#/****************************************************************************
# Waypoint Navigation: Plot library
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
2013-06-20 KJ First version
"""
# imports
import matplotlib.pyplot as plt
from pylab import ion, plot, axis, grid, title, xlabel, ylabel, draw, clf
from math import pi, sqrt
import csv

# constants
TIME = 0
MODE = 1
B_E = 2
B_N = 3
A_E = 4
A_N = 5
POSE_E = 6
POSE_N = 7
DIST_B = 8
BEARING_B = 9
HEADING_ERR = 10
DIST_AB = 11
TARGET_E = 12
TARGET_N = 13
TARGET_DIST = 14
TARGET_BEARING = 15
TARGET_HEAD_ERR = 16
LINV = 17
ANGV = 18

class ab_map():
	def __init__(self, history, window_size, easting_offset, northing_offset, buffer_seconds, save_images):
		self.rad_to_deg = 180.0/pi
        # Get parameters
		self.history = history
		self.window_size = window_size
		self.offset_e = easting_offset
		self.offset_n = northing_offset
		self.buffer_seconds = buffer_seconds
		self.save_images = save_images
		self.trkpt_threshold = 0.02 # [m]

		# Initialize map
		self.title = 'Map'
		self.pose = []
		self.wptnav = []
		self.image_count = 0
		ion() # turn interaction mode on

	def append_pose (self, time_stamp, easting, northing):
		e = easting #+ self.offset_e
		n = northing #+ self.offset_n
		if self.pose == [] or sqrt((e-self.pose[-1][1])**2 + (n-self.pose[-1][2])**2) > self.trkpt_threshold:
			self.pose.append([time_stamp, e, n])
			while time_stamp - self.buffer_seconds > self.pose[0][0]:
				self.pose.pop(0)
				
	def append_wptnav_status (self, update):
		"""
		update[B_E] += self.offset_e
		update[B_N] += self.offset_n
		update[A_E] += self.offset_e
		update[A_N] += self.offset_n
		update[POSE_E] += self.offset_e
		update[POSE_N] += self.offset_n
		update[TARGET_E] += self.offset_e
		update[TARGET_N] += self.offset_n
		"""		
		self.wptnav.append(update)
		while self.wptnav[-1][0] - self.buffer_seconds >  self.wptnav[0][0]:
			self.wptnav.pop(0)

	def update(self):
		if self.pose != []:
			plt.figure(1)
			clf()
			self.fig1 = plt.figure(num=1, figsize=(self.window_size, \
				self.window_size), dpi=80, facecolor='w', edgecolor='w')
			title (self.title)			
			xlabel('Easting [m]')
			ylabel('Northing [m]')
			axis('equal')
			grid (True)
			poseT = zip(*self.pose)	
			pose_plt = plot(poseT[1],poseT[2],'#ff0000')

			if self.wptnav != []:
				mode = self.wptnav[-1][MODE]

				if not (self.wptnav[-1][B_E] == 0 and self.wptnav[-1][B_N] == 0 and self.wptnav[-1][A_E] == 0 and self.wptnav[-1][A_N] == 0):
					b_dot = plot(self.wptnav[-1][B_E],self.wptnav[-1][B_N],'ro',markersize=8)
					a_dot = plot(self.wptnav[-1][A_E],self.wptnav[-1][A_N],'go',markersize=8)
					ab_line = plot([self.wptnav[-1][B_E],self.wptnav[-1][A_E]],[self.wptnav[-1][B_N],self.wptnav[-1][A_N]],'g')
					target_dot = plot(self.wptnav[-1][TARGET_E],self.wptnav[-1][TARGET_N],'ro',markersize=5)

				if mode == -1:
					pose_dot = plot(self.wptnav[-1][POSE_E],self.wptnav[-1][POSE_N],'b^',markersize=8)
				elif mode == 1:
					pose_dot = plot(self.wptnav[-1][POSE_E],self.wptnav[-1][POSE_N],'bs',markersize=8)
				elif mode == 2:
					pose_dot = plot(self.wptnav[-1][POSE_E],self.wptnav[-1][POSE_N],'bo',markersize=8)

		if self.save_images:
			self.fig1.savefig ('plot_map%05d.jpg' % self.image_count)
			self.image_count += 1
		draw()

	def save_last_plot(self):
		self.fig1.savefig ('plot_map.jpg')

class vel_plot():
	def __init__(self, history, window_size, buffer_seconds, save_images):
        # Get parameters
		self.history = history
		self.window_size = window_size
		self.buffer_seconds = buffer_seconds
		self.save_images = save_images

		# Initialize plot
		self.title = 'Robot'
		self.set_vel_l = []
		self.set_vel_r = []
		self.vel_l = []
		self.vel_r = []
		self.image_count = 0
		ion() # turn interaction mode on

	def append_cmd_vel_l (self, time_stamp, vel):
		self.set_vel_l.append([time_stamp, vel])
		while time_stamp - self.buffer_seconds > self.set_vel_l[0][0]:
			self.set_vel_l.pop(0)

	def append_cmd_vel_r (self, time_stamp, vel):
		self.set_vel_r.append([time_stamp, vel])
		while time_stamp - self.buffer_seconds > self.set_vel_r[0][0]:
			self.set_vel_r.pop(0)

	def append_vel_l (self, time_stamp, vel):
		self.vel_l.append([time_stamp, vel])
		while time_stamp - self.buffer_seconds > self.vel_l[0][0]:
			self.vel_l.pop(0)

	def append_vel_r (self, time_stamp, vel):
		self.vel_r.append([time_stamp, vel])
		while time_stamp - self.buffer_seconds > self.vel_r[0][0]:
			self.vel_r.pop(0)

	def update(self):
		if self.set_vel_l != []:
			plt.figure(2)
			clf()
			self.fig2 = plt.figure(num=2, figsize=(self.window_size, \
				self.window_size), dpi=80, facecolor='w', edgecolor='w')
			title (self.title)			
			xlabel('Time')
			grid (True)
			while len(self.set_vel_l) > len(self.set_vel_r):
				self.set_vel_l.pop(0)
			while len(self.set_vel_r) > len(self.set_vel_l):
				self.set_vel_r.pop(0)
			x = range(len(self.set_vel_l))
			set_vel_lT = zip(*self.set_vel_l)
			set_vel_rT = zip(*self.set_vel_r)
			l = plot (x, set_vel_lT[1],'r')
			r = plot (x, set_vel_rT[1],'g')

			if self.save_images:
				self.fig1.savefig ('vel%05d.jpg' % self.image_count)
				self.image_count += 1
			draw()


class yaw_plot():
	def __init__(self, window_size, easting_offset, northing_offset):
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
		self.window_size = window_size
		self.map_title = map_title
		self.odo = []
		self.gnss = []
		self.pose_pos = []
		self.odo_yaw = []
		self.gnss_yaw = []
		self.ahrs_yaw = []
		self.pose_yaw = []
		self.wpt_mode = 0
		self.wpt_destination = False
		self.wpt_target = False

		self.pose_image_save = True # save an image for time-lapse video generation
		self.pose_image_count = 0

		ion() # turn interaction mode on
		#if self.plot_pose:
		#	self.fig1 = plt.figure(num=1, figsize=(self.map_window_size, \
		#		self.map_window_size), dpi=80, facecolor='w', edgecolor='w')

		#if self.plot_yaw:
		#	self.fig3 = plt.figure(num=3, figsize=(map_window_size, \
		#		map_window_size), dpi=80, facecolor='w', edgecolor='w')
		#	title (map_title + ' - Orientation (yaw)')
		#	xlabel('time')
		#	ylabel('[deg]')
		#	grid (True)

	def append_pose_position (self, easting, northing):
		x = easting #+ self.offset_e
		y = northing #+ self.offset_n
		if self.pose_pos == [] or sqrt((x-self.pose_pos[-1][0])**2 + (y-self.pose_pos[-1][1])**2) > self.trkpt_threshold:
			self.pose_pos.append([x, y])

	def append_pose_yaw (self, yaw):
		self.pose_yaw.append(yaw*self.rad_to_deg)

	def set_wptnav (self, mode, dest_easting, dest_northing, target_easting, target_northing):
		self.wpt_mode = mode
		#self.wpt_destination = [dest_easting + self.offset_e, dest_northing + self.offset_n]
		#self.wpt_target = [target_easting + self.offset_e, target_northing + self.offset_n]
		self.wpt_destination = [dest_easting, dest_northing]
		self.wpt_target = [target_easting, target_northing]

	def update(self):
		if self.plot_pose and self.pose_pos != []:
			plt.figure(1)
			clf()
			self.fig1 = plt.figure(num=1, figsize=(self.map_window_size, \
				self.map_window_size), dpi=80, facecolor='w', edgecolor='w')
			title (self.map_title)			
			xlabel('Easting [m]')
			ylabel('Northing [m]')
			axis('equal')
			grid (True)
			plot (self.test_fieldT[0], self.test_fieldT[1], 'g')
			plot (self.test_fieldT[0], self.test_fieldT[1], 'go',markersize=7)

			gnssT = zip(*self.gnss)		
			gnss_plt = plot(gnssT[0],gnssT[1],'black')
		if self.plot_pose and self.pose_pos != []:
			plt.figure(1)
			poseT = zip(*self.pose_pos)		
			pose_plt = plot(poseT[0],poseT[1],'r')
		if self.plot_pose or self.plot_gnss:
			if self.wpt_mode == 1 or self.wpt_mode == 2:
				if self.wpt_mode > 0 and self.wpt_destination != False:
					dest_plt = plot(self.wpt_destination[0],self.wpt_destination[1],'ro',markersize=8)
				if self.wpt_mode == 1:
					if self.pose_pos != [] and (self.pose_pos[-1][0] != 0.0  or self.pose_pos[-1][1] != 0.0):
						pose_plt = plot(self.pose_pos[-1][0],self.pose_pos[-1][1],'bs',markersize=8)
					if self.wpt_target != False:
						target_plt = plot(self.wpt_target[0],self.wpt_target[1],'ro',markersize=5)
				elif self.wpt_mode == 2:
					if self.pose_pos != [] and (self.pose_pos[-1][0] != 0.0  or self.pose_pos[-1][1] != 0.0):
						pose_plt = plot(self.pose_pos[-1][0],self.pose_pos[-1][1],'bo',markersize=8)
					if self.wpt_target != False:
						target_plt = plot(self.wpt_target[0],self.wpt_target[1],'ro',markersize=5)
			elif self.wpt_mode == -1:
				if self.pose_pos != [] and (self.pose_pos[-1][0] != 0.0  or self.pose_pos[-1][1] != 0.0):
					pose_plt = plot(self.pose_pos[-1][0],self.pose_pos[-1][1],'b^',markersize=8)

		if self.pose_image_save:
			self.fig1.savefig ('img%05d.jpg' % self.pose_image_count)
			self.pose_image_count += 1

		if self.plot_odometry and self.odo != []:
			plt.figure(2)
			odoT = zip(*self.odo)		
			odo_plt = plot(odoT[0],odoT[1],'b')
		if self.plot_yaw:

			self.fig3 = plt.figure(num=3, figsize=(map_window_size, \
				map_window_size), dpi=80, facecolor='w', edgecolor='w')
			title (map_title + ' - Orientation (yaw)')
			xlabel('time')
			ylabel('[deg]')
			grid (True)


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

