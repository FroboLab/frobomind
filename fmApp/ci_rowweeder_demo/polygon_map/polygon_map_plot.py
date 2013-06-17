#!/usr/bin/env python
#*****************************************************************************
# Polygon Map Plot
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
2013-05-31 KJ First version
"""

# imports
import matplotlib.pyplot as plt
from pylab import ion, plot, axis, grid, title, xlabel, ylabel, draw

class polygon_map_plot():
	def __init__(self, map_title, map_window_size, easting_offset, northing_offset):
		self.offset_e = easting_offset # [m]
		self.offset_n = northing_offset # [m]
		self.nearby_threshold = 10.0 # [m]
		self.polypts = []
		self.within = []
		self.nearby = []
		self.polyplt = []

		ion() # turn interaction mode on
		self.fig1 = plt.figure(num=1, figsize=(map_window_size, \
			map_window_size), dpi=80, facecolor='w', edgecolor='w')
		title (map_title)
		xlabel('Easting [m]')
		ylabel('Northing [m]')
		axis('equal')
		grid (False)

	def add_polygon (self, polygon):
		for i in range(len(polygon)):
			polygon[i][0] += self.offset_e
			polygon[i][1] += self.offset_n
		polygon.append (polygon[0]) # close the polygon for plotting (must called be after modifying the polygon!)
		polygonT =  zip(*polygon)
		self.polypts.append (polygonT)
		self.polyplt.append (plot(self.polypts[-1][0], self.polypts[-1][1], 'black'))

	def draw_polygon_within (self, num):
		self.polyplt[num] = plot(self.polypts[num][0], self.polypts[num][1], 'red')

	def draw_polygon_outside (self, num):
		self.polyplt[num] = plot(self.polypts[num][0], self.polypts[num][1], 'black')

	def update_map_plot (self):
		if len(self.polypts) > 0:
			plt.figure(1)
			draw()
		pass

	def save_map_plot (self):
		self.fig1.savefig ('polygon_map_plot.png')

	def update_pos (self):
		pass

