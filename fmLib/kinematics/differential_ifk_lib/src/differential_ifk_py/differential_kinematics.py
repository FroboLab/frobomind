#!/usr/bin/env python
#/****************************************************************************
# Differential_kinematics
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
This library contains functions for forward and inverse kinematics of a 
differentially steered vehicle.

The forward kinematic model describes how the speed of the two wheels
(vel_right, vel_left) translate into a forward speed (vel_lin) and an angular
speed (vel_ang).

The inverse kinematic model describes how the speed of the wheels
(vel_left, vel_right) must be set for the robot to obtain a forward speed
(vel_lin) and an angular speed (vel_ang).

2013-10-05 Kjeld Jensen: First version
2014-05-03 Kjeld Jensen: Corrected an annoying bug in the function inverse()
"""

# imports

class differential_kinematics():
	def __init__(self, wheel_distance):
		self.wheel_dist = wheel_distance
		self.wheel_dist_half = wheel_distance/2.0
		self.vel_lin = 0.0 
		self.vel_ang = 0.0 
		self.vel_left = 0.0 
		self.vel_right = 0.0 

	def forward (self, vel_left, vel_right):
		self.vel_lin = (vel_right + vel_left)/2.0 # [m/s]
		self.vel_ang = (vel_right - vel_left)/self.wheel_dist # [rad/s]
		return (self.vel_lin, self.vel_ang)

	def inverse (self, vel_lin, vel_ang):
		self.vel_left  = vel_lin - self.wheel_dist_half*vel_ang # [m/s]
		self.vel_right = vel_lin + self.wheel_dist_half*vel_ang # [m/s]
		return (self.vel_left, self.vel_right) 

		
