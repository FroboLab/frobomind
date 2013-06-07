#!/usr/bin/env python
#/****************************************************************************
# Waypoint Navigation
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

Navigation modes:
  STWP: Straight to waypoint B
  MCTE: Minimize cross track error from A to B


2013-06-06 KJ First version
"""

# imports
from math import pi, atan2, sqrt, fabs
from pid_controller import pid_controller

WPT_E = 0
WPT_N = 1
WPT_ID = 2
WPT_MODE = 3
WPT_TOLERANCE = 4
WPT_SPEED = 5

STATE_STOP = 0
STATE_TURN_INIT = 1
STATE_TURN = 2
STATE_DRIVE_INIT = 3
STATE_DRIVE = 4

class waypoint_navigation():
	def __init__(self):
		self.pi2 = 2.0*pi
		self.linear_speed = 0.0
		self.angular_speed = 0.0
		self.a = False
		self.b = False
		self.state = STATE_STOP
		self.dist = 0.0
		self.bearing = 0.0
		self.heading_err = 0.0 
		self.pose = False
		self.UPDATE_NONE = 0
		self.UPDATE_ARRIVAL = 1
		self.print_interval = 10
		self.print_count = 0
		self.turn_acceptable_heading_err = 20.0*pi/180.0
		self.turn_stop_at_heading_err = 3.0*pi/180.0
		self.turn_max_speed = pi/4.0
		self.pid = pid_controller () # initialize PID controller class

	def pose_update (self, easting, northing, yaw):
		self.pose = [easting, northing, yaw]

	def navigate (self, destination, origin): # easting, northing
		self.b = destination
		if origin != False:
			self.a = origin
		else:
			self.a = self.pose	
		self.state = STATE_DRIVE_INIT

	def stop (self):
		print "Stop"
		self.a = False
		self.b = False
		self.state = STATE_STOP

	def turn_init(self):
		print 'Turning to adjust heading error: %.1f' %(self.heading_err*180/pi)
		dT = 0.1
		Kp = 1.1
		Ki = 0.3
		Kd = 0.0
		integral_max = 1
		integral_min = 0
		self.pid.reset (dT, Kp, Ki, Kd, integral_max, integral_min)
		self.state = STATE_TURN

	def turn(self):
		if fabs(self.heading_err) < self.turn_stop_at_heading_err:
			self.state = STATE_DRIVE_INIT
		else:
			pid_u = self.pid.update (self.heading_err) # get controller output

			if pid_u > self.turn_max_speed: # make sure we limit ourselves to max propulsion
				pid_u = self.turn_max_speed
			elif pid_u < -self.turn_max_speed:
				pid_u = -self.turn_max_speed		
			self.linear_speed = 0.0
			self.angular_speed = pid_u

	def drive_init(self):
		print "Driving towards wpt"
		dT = 0.1
		Kp = 1.5
		Ki = 0.5
		Kd = 0.0
		integral_max = 1
		integral_min = 0
		self.pid.reset (dT, Kp, Ki, Kd, integral_max, integral_min)
		self.state = STATE_DRIVE

	def drive(self):
		if fabs(self.heading_err) > self.turn_acceptable_heading_err:
			self.state = STATE_TURN_INIT
		else:
			pid_u = self.pid.update (self.heading_err) # get controller output
			if pid_u > self.turn_max_speed: # make sure we limit ourselves to max propulsion
				pid_u = self.turn_max_speed
			elif pid_u < -self.turn_max_speed:
				pid_u = -self.turn_max_speed		
			self.linear_speed = 0.7
			self.angular_speed = pid_u

	def update(self):
		status = self.UPDATE_NONE
		if self.state != STATE_STOP and self.pose != False: # if we have a valid pose
			self.dist = sqrt((self.b[0]-self.pose[0])**2 + (self.b[1]-self.pose[1])**2) # distance to b
			easting_vector_component = self.b[0] - self.pose[0]  # bearing (angle with easting axis) to b
			northing_vector_component = self.b[1] - self.pose[1]
			self.bearing = self.angle_limit(atan2 (northing_vector_component, easting_vector_component)) 
			self.heading_err = self.angle_diff(self.bearing, self.pose[2]) # signed difference from heading to bearing

			if self.state == STATE_DRIVE_INIT:
				self.drive_init()
			elif self.state == STATE_DRIVE:
				self.drive()
			elif self.state == STATE_TURN_INIT:
				self.turn_init()
			elif self.state == STATE_TURN:
				self.turn()

			if self.dist <= self.b[WPT_TOLERANCE]:
				status = self.UPDATE_ARRIVAL 

			self.print_count += 1
			if self.print_count % self.print_interval == 0:
				print "dist: %.2f bearing: %.1f heading err: %.1f" % (self.dist, self.bearing*180.0/pi, self.heading_err*180.0/pi)

		return (status, self.linear_speed, self.angular_speed)
	
	def angle_limit (self, angle): # return angle within [0;2pi[
		while angle < 0:
			angle += self.pi2
		while angle >= self.pi2:
			angle -= self.pi2
		return angle

	# return signed difference between new and old angle
	def angle_diff (self, angle_new, angle_old):
		diff = angle_new - angle_old
		while diff < -pi:
			diff += self.pi2
		while diff > pi:
			diff -= self.pi2
		return diff

