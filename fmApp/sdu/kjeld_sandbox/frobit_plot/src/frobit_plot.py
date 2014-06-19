#!/usr/bin/env python
#/****************************************************************************
# FroboMind - Frobit plot functions
# Copyright (c) 2013-2014, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
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
2014-02-19 KJ Migrated to a frobit_plot component
2014-04-19 KJ Added support for reversing the avatar
2014-06-19 KJ Various bug fixes
"""
# imports
import matplotlib.pyplot as plt
from pylab import ion, plot, axis, grid, title, xlabel, ylabel, draw, clf
from math import pi, sqrt, sin, cos
import csv

class frobit_plot():
	def __init__(self, plot_pose_track, plot_gnss_track, plot_odometry_track, plot_yaw, map_trackpoint_threshold, map_max_trackpoints, map_minimum_size, map_easting_offset, map_northing_offset, map_update_frequency, map_title, map_window_size, avatar_extension_front, avatar_extension_rear, avatar_extension_lateral, avatar_reverse):
		self.rad_to_deg = 180.0/pi
		self.deg_to_rad = pi/180.0
		self.save_time_lapse_images = False

        # Get parameters
		self.plot_pose_track = plot_pose_track
		self.plot_gnss_track = plot_gnss_track
		self.plot_odometry_track = plot_odometry_track
		self.plot_yaw = plot_yaw
		self.map_trackpoint_threshold = map_trackpoint_threshold
		self.map_max_trackpoints = map_max_trackpoints
		self.map_minimum_size = map_minimum_size
		self.map_easting_offset = map_easting_offset
		self.map_northing_offset = map_northing_offset
		self.map_update_frequency = map_update_frequency
		self.map_title = map_title
		self.map_window_size = map_window_size
		self.avatar_extension_front = avatar_extension_front
		self.avatar_extension_rear = avatar_extension_rear
		self.avatar_extension_lateral = avatar_extension_lateral
		self.avatar_reverse = avatar_reverse
     
		# Initialize map
		self.map_image_cnt = 0
		self.gnss = []
		self.odo_pos = []
		self.gnss_pos = []
		self.pose_pos = []
		self.odo_yaw = []
		self.pose_yaw = []
		self.wpt_mode = 0
		self.wpt_a = False
		self.wpt_b = False
		self.wpt_target = False
		self.wpt_first = True
		af = avatar_extension_front		
		ar = -avatar_extension_rear		
		al = avatar_extension_lateral		
		ac = (af+(-ar)) * 0.2
	
		if self.avatar_reverse == False:
			self.avatar = [[af-ac,al],[af,al-ac],[af,-al+ac],[af-ac,-al],[ar,-al],[ar,al],[af-ac,al]] # Frobit
		else:
			self.avatar = [[af,al],[af,-al],[ar+ac,-al],[ar,-al+ac],[ar,al-ac],[ar+ac,al],[af,al]] # Frobit reversed

		ion() # turn interaction mode on
		if self.plot_gnss_track or self.plot_pose_track:
			self.fig1 = plt.figure(num=1, figsize=(self.map_window_size, \
				self.map_window_size), dpi=80, facecolor='w', edgecolor='w')

		if self.plot_odometry_track:
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
		x = easting + self.map_easting_offset
		y = northing + self.map_northing_offset
		if self.pose_pos == [] or sqrt((x-self.pose_pos[-1][0])**2 + (y-self.pose_pos[-1][1])**2) > self.map_trackpoint_threshold:
			self.pose_pos.append([x, y])
			if len (self.pose_pos) > self.map_max_trackpoints:
				del (self.pose_pos[0])

	def append_gnss_position (self, easting, northing):
		x = easting + self.map_easting_offset
		y = northing + self.map_northing_offset
		if self.gnss_pos == [] or sqrt((x-self.gnss_pos[-1][0])**2 + (y-self.gnss_pos[-1][1])**2) > self.map_trackpoint_threshold:
			self.gnss_pos.append([x, y])
			if len (self.gnss_pos) > self.map_max_trackpoints:
				del (self.gnss_pos[0])

	def append_odometry_position (self, x, y):
		if self.odo_pos == [] or sqrt((x-self.odo_pos[-1][0])**2 + (y-self.odo_pos[-1][1])**2) > self.map_trackpoint_threshold:
			self.odo_pos.append([x, y])
			if len (self.odo_pos) > self.map_max_trackpoints:
				del (self.odo_pos[0])

	def append_pose_yaw (self, yaw):
		self.pose_yaw.append(yaw*self.rad_to_deg)

	def append_odo_yaw (self, yaw):
		self.odo_yaw.append(yaw*self.rad_to_deg)

	def set_wptnav (self, mode, a_easting, a_northing, b_easting, b_northing, target_easting, target_northing):
		self.wpt_mode = mode
		self.wpt_a = [a_easting + self.map_easting_offset, a_northing + self.map_northing_offset]
		self.wpt_b = [b_easting + self.map_easting_offset, b_northing + self.map_northing_offset]
		self.wpt_target = [target_easting + self.map_easting_offset, target_northing + self.map_northing_offset]

	def update(self):
		if self.plot_gnss_track or self.plot_pose_track:
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

			# plot square to define minimim map size
			if self.map_minimum_size > 0.1 and self.pose_yaw != [] and self.pose_pos != []:
				pe = self.pose_pos[-1][0]
				pn = self.pose_pos[-1][1]

				mms = self.map_minimum_size/2.0
				mms_list = [[pe-mms, pn+mms],[pe-mms, pn-mms],[pe+mms, pn-mms],[pe+mms, pn+mms]]
				mms_listT = zip(*mms_list)
				mms_plt = plot (mms_listT[0],mms_listT[1],'w')

		# If A-B navigating draw the navigation line
		if self.plot_pose_track and (self.wpt_mode == 1 or self.wpt_mode == 2):
			if self.wpt_mode > 0 and self.wpt_b != False:
				#a_plt = plot(self.wpt_a[0],self.wpt_a[1],'go',markersize=4)
				#b_plt = plot(self.wpt_b[0],self.wpt_b[1],'ro',markersize=4)
				ab_plt  =  plot([self.wpt_a[0], self.wpt_b[0]], [self.wpt_a[1], self.wpt_b[1]],'#777777')
			if self.wpt_mode == 1:
				if self.pose_pos != [] and (self.pose_pos[-1][0] != 0.0  or self.pose_pos[-1][1] != 0.0):
					#pose_plt = plot(self.pose_pos[-1][0],self.pose_pos[-1][1],'bs',markersize=6)
					pass
				if self.wpt_target != False:
					pass
					target_plt = plot(self.wpt_target[0],self.wpt_target[1],'ro',markersize=4)
			elif self.wpt_mode == 2:
				if self.pose_pos != [] and (self.pose_pos[-1][0] != 0.0  or self.pose_pos[-1][1] != 0.0):
					#pose_plt = plot(self.pose_pos[-1][0],self.pose_pos[-1][1],'bs',markersize=6)
					pass
				if self.wpt_target != False:
					target_plt = plot(self.wpt_target[0],self.wpt_target[1],'ro',markersize=4)

		# draw the GNSS track
		if (self.plot_gnss_track and self.gnss_pos != []):
			gnssT = zip(*self.gnss_pos)		
			gnss_plt = plot(gnssT[0],gnssT[1],'#000000')

		# draw the pose track
		if self.plot_pose_track and self.pose_pos != []:
			poseT = zip(*self.pose_pos)		
			pose_plt = plot(poseT[0],poseT[1],'#ff0000')

		# draw the avatar
		if self.plot_pose_track and self.pose_yaw != [] and self.pose_pos != []:
			yaw = self.pose_yaw[-1]*self.deg_to_rad
			pe = self.pose_pos[-1][0]
			pn = self.pose_pos[-1][1]

			# define avatar plot
			avatar_plot = []
			for i in xrange(len(self.avatar)):
				c = self.vec2d_rot (self.avatar[i],yaw)
				avatar_plot.append([c[0]+pe,c[1]+pn])
			avatar_plotT = zip(*avatar_plot)

			if self.wpt_mode == 1:
				ava_plt = plot(avatar_plotT[0],avatar_plotT[1],'b')
			elif self.wpt_mode == 2:
				ava_plt = plot(avatar_plotT[0],avatar_plotT[1],'r')
			else:
				ava_plt = plot(avatar_plotT[0],avatar_plotT[1],'black')

		if self.save_time_lapse_images == True:
			self.fig1.savefig ('img%05d.jpg' % self.map_image_cnt)
			self.map_image_cnt += 1

		if self.plot_odometry_track and self.odo_pos != []:
			plt.figure(2)
			odoT = zip(*self.odo_pos)		
			odo_plt = plot(odoT[0],odoT[1],'b')
		if self.plot_yaw:
			if  self.odo_yaw != []:
				plt.figure(3)
				odo_yaw_plt = plot(self.odo_yaw,'b')
			if  self.pose_yaw != []:
				plt.figure(3)
				pose_yaw_plt = plot(self.pose_yaw,'r')
		if self.plot_gnss_track or self.plot_pose_track or self.plot_odometry_track or self.plot_yaw:
			draw()

	def save(self, file_name):
		if self.plot_gnss_track or self.plot_pose_track:
			if self.plot_gnss_track and self.plot_pose_track:
				self.fig1.savefig (file_name+'-gnss-pose.png')
			elif self.plot_gnss_track:
				self.fig1.savefig (file_name+'-gnss.png')
			else:
				self.fig1.savefig (file_name+'-pose.png')
		if self.plot_odometry_track:
			self.fig2.savefig (file_name+'-odometry.png')
		if self.plot_yaw:
			self.fig3.savefig (file_name+'-yaw.png')

