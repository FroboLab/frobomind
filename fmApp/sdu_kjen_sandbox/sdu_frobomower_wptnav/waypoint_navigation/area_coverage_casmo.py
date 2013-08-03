#!/usr/bin/env python
#/****************************************************************************
# Area Coverage CASMO library
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
This area coverage library implements a coverage algorithm originally
developed for the CASMOBOT robot at the University of Southern Denmark.

In this library the East-North-Up (ENU) coordinate system is used as
reference to allow use of GNSS based coordinates represented in a
Transverse Mercator projection. A position is defined by
[easting, northing] and an orientation is defined by [yaw]. Positive
rotation is counter clockwise with origo at the easting axis.

The library does not perform any error checking, so functions must be 
called in the correct order.

2013-08-02 KJ First version
"""

# imports
from math import sin, cos, pi, sqrt

# defines
default_length = 100.0 # [m]
default_width = 1.0 # [m]

class area_coverage_casmo():
	def __init__(self):
		self.S_IDLE = 0
		self.S_MOVE_FIRST = 1
		self.S_TURN_LEFT = 2
		self.S_MOVE_LEFT = 3
		self.S_TURN_RIGHT = 4
		self.S_MOVE_RIGHT = 5
		self.state = self.S_IDLE
		self.width = default_width
		self.left = False
		self.right = False
	
	def start(self, pos, bearing):
		if self.state == self.S_IDLE:
			self.state = self.S_MOVE_FIRST
			self.length = default_length
			self.left = False
			self.right = False
			# define navigation vector
			v = self.vec2d_rot([default_length, 0], bearing)
			# return current pos plus navigation vector
			return (self.vec2d_add([pos[0], pos[1]], v))
		else:
			return False

	def turn_left(self, pos):
		if self.state == self.S_MOVE_FIRST or self.state == S_MOVE_RIGHT:
			# determine point on current navigation vector closest to pos
		
			pt = 

			if self.left != False:
				self.width = sqrt((pt[0]-self.left[0])**2 + (pt[1]-self.left[1])**2)
			# determine the beginning of the next navigation vector
			self.right = 
			return ([0,0])
	
	def turn_right(self, pos):
		if self.state == self.S_MOVE_FIRST or self.state == S_MOVE_LEFT:
			return ([0,0])

	def goto_next(self):
		return ([init_e, init_n, init_bearing])

	def reset_length(self):
		return ([init_e, init_n, init_bearing])

	def stop(self):
		self.state = self.S_IDLE
	
	def param_set_default_length(self, length):
		self.length = length

	def param_set_width(self, width):
		self.width = width

	def vec2d_add (self, v1, v2): # return v1+v2
		return ([v1[0]+v2[0], v1[1]+v2[1]])

	def vec2d_rot (self, v, theta): # return the vector v rotated by theta
		rot_x = v[0]*cos(theta) - v[1]*sin(theta)
		rot_y = v[0]*sin(theta) + v[1]*cos(theta)
		return ([rot_x, rot_y])

	def angle_limit (self, angle): # return angle within [0;2pi[
		while angle < 0:
			angle += self.pi2
		while angle >= self.pi2:
			angle -= self.pi2
		return angle

casmo = area_coverage_casmo()
casmo.param_set_width (0.8)
print casmo.start([0.0, 0.0], pi/2.0)
print casmo.turn_left ([50.0, -1.0])

