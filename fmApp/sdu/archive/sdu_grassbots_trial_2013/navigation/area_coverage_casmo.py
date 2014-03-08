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

2013-08-06 KJ First version
2013-09-22 KJ Corrected a bug that caused the default length to always be 100m
              Added supportt for some launch parameters
"""

# imports
from math import sin, cos, pi, sqrt

# defines
default_length = 100.0 # [m]
default_width = 1.0 # [m]

E = 0
N = 1

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
        self.default_length = default_length
        self.left = False
        self.right = False
        self.a = False
        self.b = False
        self.ab_norm = False
	
    def start(self, pos, bearing):
        if self.state == self.S_IDLE:
            self.state = self.S_MOVE_FIRST
            self.length = self.default_length
            self.left = False
            self.right = False
            self.a = pos
            # define bearing norm vector
            self.ab_norm = self.vec2d_rot([1, 0], bearing)
            # define destination waypoint (current pos plus navigation vector)
            self.b = self.vec2d_add(pos, self.vec2d_multiply(self.ab_norm, self.default_length))
            return (self.b)
        else:
            return False

    def turn_left(self, pos):
        if self.state == self.S_MOVE_FIRST or self.state == self.S_MOVE_RIGHT:
            # find the closest point along the ab line which will be our new right limit
            self.b = self.closest_pt_on_ab_line (pos)
            # determine new length of the coverage lines
            if self.left != False:
                self.length = self.vec2d_pt_distance (self.left, self.b)
            # since we obviously just arrived at this new right limit we initiate a turn left
            self.state = self.S_MOVE_RIGHT
            return (self.goto_next())
            
    def turn_right(self, pos):
        if self.state == self.S_MOVE_FIRST or self.state == self.S_MOVE_LEFT:
            # find the closest point along the ab line which will be our new left limit
            self.b = self.closest_pt_on_ab_line (pos)
            # determine new length of the coverage lines
            if self.right != False:
                self.length = self.vec2d_pt_distance (self.right, self.b)
            # since we obviously just arrived at this new left limit we initiate a turn right
            self.state = self.S_MOVE_LEFT
            return (self.goto_next())

    def goto_next(self):
        if self.state == self.S_TURN_LEFT:
            self.a = self.b
            self.right = self.a
            self.ab_norm = self.vec2d_rot (self.ab_norm, pi/2.0)
            nav = self.vec2d_multiply (self.ab_norm, self.length)
            self.b = self.vec2d_add (self.a, nav)
            self.left = self.b
            self.state = self.S_MOVE_LEFT                        
                        
        elif self.state == self.S_MOVE_LEFT:
            self.a = self.b
            self.ab_norm = self.vec2d_rot (self.ab_norm, -pi/2.0)
            nav = self.vec2d_multiply (self.ab_norm, self.width)
            self.b = self.vec2d_add(self.a, nav)                                      
            self.left = self.b
            self.state = self.S_TURN_RIGHT

        elif self.state == self.S_TURN_RIGHT:
            self.a = self.b
            self.ab_norm = self.vec2d_rot (self.ab_norm, -pi/2.0)
            nav = self.vec2d_multiply (self.ab_norm, self.length)
            self.b = self.vec2d_add (self.a, nav)
            self.right = self.b
            self.state = self.S_MOVE_RIGHT                        

        elif self.state == self.S_MOVE_RIGHT:
            self.a = self.b
            self.ab_norm = self.vec2d_rot (self.ab_norm, pi/2.0)
            nav = self.vec2d_multiply (self.ab_norm, self.width)
            self.b = self.vec2d_add(self.a, nav)                                      
            self.right = self.b
            self.state = self.S_TURN_LEFT

        else:
            self.b = False       
        return (self.b)

    def reset_length(self):
        if self.state == self.S_MOVE_LEFT or self.state == self.S_MOVE_RIGHT:
            self.length = self.default_length
            nav = self.vec2d_multiply (self.ab_norm, self.length) # update navigation vector
            self.b = self.vec2d_add(self.a, nav) # calculate new waypoint
            # forget that we know the reset limit
            if self.state == self.S_MOVE_LEFT:
                self.left = False
            else:
                self.right = False
            return (self.b)
        else:
            return (False)

    def stop(self):
        self.state = self.S_IDLE
	
    def param_set_default_length(self, length):
        self.default_length = length

    def param_set_width(self, width):
        self.width = width

    def vec2d_add (self, v1, v2): # return v1+v2
        return ([v1[0]+v2[0], v1[1]+v2[1]])

    def vec2d_multiply (self, v, scalar): # return v*s
        return ([v[0]*scalar, v[1]*scalar])

    def vec2d_rot (self, v, theta): # return the vector v rotated by theta
        rot_x = v[0]*cos(theta) - v[1]*sin(theta)
        rot_y = v[0]*sin(theta) + v[1]*cos(theta)
        return ([rot_x, rot_y])

    def vec2d_pt_distance (self, a, b):
        return (sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2))

    def closest_pt_on_ab_line (self, pos):
        # find distance to the point on the ab-line that is closest to pos
        self.a_dot_ab_norm = self.a[E]*self.ab_norm[E] + self.a[N]*self.ab_norm[N] # (vector defined by point a) dot (normalized ab vector) 
        pos_dot_ab_norm = pos[E]*self.ab_norm[E] + pos[N]*self.ab_norm[N] # (vector defined by pos) dot (normalized ab vector)
        d = pos_dot_ab_norm - self.a_dot_ab_norm  # distance along the ab line

        # determine the closest point along the ab line
        pt = [self.a[E] + d*self.ab_norm[E], self.a[N] + d*self.ab_norm[N]]
        return (pt)

    def angle_limit (self, angle): # return angle within [0;2pi[
        while angle < 0:
            angle += self.pi2
        while angle >= self.pi2:
            angle -= self.pi2
        return angle

