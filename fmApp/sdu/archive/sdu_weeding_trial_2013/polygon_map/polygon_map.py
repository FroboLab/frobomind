#!/usr/bin/env python
#*****************************************************************************
# Polygon Map
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
2013-06-10 KJ First version
"""

from math import sqrt

class polygon_map():
	def __init__(self):
		# parameters
		self.max_speed = 2.0 # [m/s]
		self.nearby_threshold = 5.0 # [m]
		self.max_iteration_time = 2.0 # [s]
		self.polygons_per_update = 1600
		self.reset_threshold = 10.0 # [m] reset map timeouts if pos switches more than this from one to another

		# internal vars
		self.poly_pts = [] # polygon coordinates
		self.poly_id = [] # name (text)
		self.poly_bounds = [] # [emin, emax, nmin, nmax]
		self.poly_timeout = [] # when to update next time 
		self.poly_nearby = [] # True if pos is currently nearby the polygon
		self.poly_inside = [] # True if pos is currently inside the polygon
		self.poly_total = 0 # number of polygons

		self.nearby = [] # list of references to nearby polygons
		self.upd_range = range(self.polygons_per_update)
		self.upd = 0 # currently updating this one
		self.easting = 0.0
		self.northing = 0.0

	# set the (easting/northing) nearby threshold
	def set_nearby_threshold (self, nearby_threshold):
		self.nearby_threshold = nearby_threshold
	
	# set the number of polygons to iterate for each update
	def set_polygons_per_update (self, polygons_per_update):
		self.polygons_per_update = polygons_per_update
		self.upd_range = range(self.polygons_per_update)

	def add_polygon (self, identity, polygon):
		self.poly_id.append (identity)
		self.poly_pts.append (polygon)
		self.poly_timeout.append (0.0)
		self.poly_nearby.append (False)
		self.poly_inside.append (False)
		emin = nmin = 20000000.0
		emax = nmax = -20000000.0
		for i in xrange(len(polygon)):
			if polygon[i][0] < emin:
				emin = polygon[i][0]
			if polygon[i][0] > emax:
				emax = polygon[i][0]
			if polygon[i][1] < nmin:
				nmin = polygon[i][1]
			if polygon[i][1] > nmax:
				nmax = polygon[i][1]
		self.poly_bounds.append ([emin, emax, nmin, nmax])
		self.poly_total += 1

	def pos_in_polygon (self, poly_num): # http://www.ariel.com.au/a/python-point-int-poly.html
		l = len(self.poly_pts[poly_num])
		inside = 0  
		p1e,p1n = self.poly_pts[poly_num][0]
		for i in range(l+1):
		    p2e,p2n = self.poly_pts[poly_num][i % l]
		    if self.northing > min(p1n,p2n):
		        if self.northing <= max(p1n,p2n):
		            if self.easting <= max(p1e,p2e):
		                if p1n != p2n:
		                    einters = (self.northing-p1n)*(p2e-p1e)/(p2n-p1n)+p1e
		                if p1e == p2e or self.easting <= einters:
		                    inside = not inside
		    p1e,p1n = p2e,p2n
		return inside

	def update_map (self, time):
		# check the next (self.polygons_per_update) polygons in the polygon list
		for i in self.upd_range:
			self.upd += 1
			if self.upd == self.poly_total: # reset if at the end
				self.upd = 0
			# test the polygon
			if self.poly_nearby[self.upd] == True or time >= self.poly_timeout[self.upd]:
				# calculate distance to polygon bounding box
				if self.easting < self.poly_bounds[self.upd][0]: 				
					e_dist = self.poly_bounds[self.upd][0] - self.easting
				elif self.easting > self.poly_bounds[self.upd][1]:
					e_dist = self.easting - self.poly_bounds[self.upd][1]
				else:
					e_dist = 0.0

				if self.northing < self.poly_bounds[self.upd][2]: 				
					n_dist = self.poly_bounds[self.upd][2] - self.northing
				elif self.northing > self.poly_bounds[self.upd][3]:
					n_dist = self.northing - self.poly_bounds[self.upd][3]
				else:
					n_dist = 0.0

				if e_dist + n_dist < 0.001: # pos is inside min-max ranges
					dist = 0.0
				else:
					dist = sqrt(e_dist**2 + n_dist**2)
				
				# update 'nearby' list based on nearby threshold
				if dist <= self.nearby_threshold: # if distance within threshold
					if self.poly_nearby[self.upd] == False:
						self.poly_nearby[self.upd] = True
						self.nearby.append(self.upd)

				else: # if distance outside threshold
					if self.poly_nearby[self.upd] == True:
						self.poly_timeout[self.upd] = time + dist/self.max_speed - self.max_iteration_time
						self.poly_nearby[self.upd] = False
						listpos = self.nearby.index(self.upd)
						self.nearby.pop(listpos)

		changes = []
		# check all nearby polygons using point in polygon algorithm
		for i in xrange(len(self.nearby)):
			poly_num = self.nearby[i]
			inside = self.pos_in_polygon(poly_num)
			if inside == True and self.poly_inside[poly_num] == False:
				self.poly_inside[poly_num] = True
				changes.append (poly_num+1)
			elif inside == False and self.poly_inside[poly_num] == True:
				self.poly_inside[poly_num] = False
				changes.append (-poly_num-1)

		return changes

	# set timeout for all polygons to enforce a quick update
	def reset_map (self):
		for i in xrange(self.poly_total):
			self.poly_timeout[i] = 0.0

	# update the position
	def update_pos (self, easting, northing):
		if abs(easting-self.easting) > self.reset_threshold or abs(northing-self.northing) > self.reset_threshold:
			self.reset_map()
		self.easting = easting
		self.northing = northing

