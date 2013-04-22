#!/usr/bin/env python
#/****************************************************************************
# Pose 2D Estimator Library Plot
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

class estimatorplot():
	def __init__(self):
        # Get parameters
		self.map_title = 'Pose 2D Estimator'
		self.map_window_size = 5.0 # [inches]
		self.trkpt_threshold = 0.1 # [m]
        
		# Initialize map
		self.origoX = 0
		self.origoY = 0
		self.odo = []
		self.track = []

		ion() # turn interaction mode on
		self.fig1 = plt.figure(num=1, figsize=(self.map_window_size, \
			self.map_window_size), dpi=80, facecolor='w', edgecolor='w')
		title (self.map_title)
		xlabel('Easting [m]')
		ylabel('Northing [m]')
		axis('equal')
		grid (True)

	def append_odo (self, x, y):
		self.odo.append([x, y])

	def append_pos (self):
		x = msg.pose.pose.position.x - self.origoX
		y = msg.pose.pose.position.y - self.origoY
		if (abs(x-self.track[-1][0]) > self.trkpt_threshold \
			or abs(y-self.track[-1][1]) > self.trkpt_threshold):
			self.track.append([x, y])

	def update_plot(self):
		plt.figure(1)
		if self.odo != []:
			odoT = zip(*self.odo)		
			odo_plt = plot(odoT[0],odoT[1],'r')
		draw()

	def save_plot(self):
		self.fig1.savefig ('plot.png')

