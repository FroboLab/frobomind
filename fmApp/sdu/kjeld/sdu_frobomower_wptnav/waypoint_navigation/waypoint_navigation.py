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
This waypoint navigation class implements an AB navigation controller

Look at the update function to get an idea of how it works.

Navigation modes:
  STWP: Straight to destination (b) waypoint
  MCTE: Minimize cross track error from origin (a) to destination (b)

2013-06-07 KJ First version
"""

# imports
from math import pi, atan2, sqrt, fabs
from pid_controller import pid_controller

# list index for destination (b) and origin (a) waypoints
WPT_E = 0
WPT_N = 1
WPT_ID = 2
WPT_MODE = 3
WPT_TOLERANCE = 4
WPT_SPEED = 5

class waypoint_navigation():
	def __init__(self, update_rate, debug):
		# constants
		self.UPDATE_NONE = 0
		self.UPDATE_ARRIVAL = 1
		self.pi2 = 2.0*pi
		self.deg_to_rad = pi/180.0
		self.rad_to_deg = 180.0/pi

		# parameters
		self.update_rate = update_rate # [Hz]
		self.update_interval = (1.0/self.update_rate) # [s]
		self.wpt_linear_speed_default = 0.7 # [m/s]
		self.linear_speed_max = 1.0 # [m/s]
		self.wpt_linear_ramp_down_speed_default = 0.3 # [m/s]
		self.wpt_linear_ramp_down_at_dist_default = 1.0 # [m]
		self.angular_speed_max = 0.45 # [radians/s]
		self.turn_start_at_heading_err = 17.0*self.deg_to_rad # [radians] set to 2pi if not applicable to the robot
		self.turn_acceptable_heading_err = 2.0*self.deg_to_rad # [radians]
		self.drive_kp =23.0
		self.drive_ki = 0.0
		self.drive_kd = 2.4
		self.drive_integral_max = 1.0
		self.turn_kp = 2.5
		self.turn_ki = 0.0
		self.turn_kd = 1.0
		self.turn_integral_max = 1.0
		self.target_ahead = 2.0 # [m] the intermediate target is along the ab line 'self.target_ahead' meters ahead of the pose 
		self.target_percent = 0.5 # [0;1] when distance to b is less than self.target_ahead, the target is self.target_percent times the distance to b ahead of the pose
		self.wpt_tolerance_default = 0.5 # [m]
		self.print_interval = 10

		# navigation controller state machine
		self.STATE_STOP = 0
		self.STATE_STANDBY = 1
		self.STATE_DRIVE_INIT = 2
		self.STATE_DRIVE = 3
		self.STATE_TURN_INIT = 4
		self.STATE_TURN = 5

		self.state = self.STATE_STOP
		self.prev_state = self.STATE_STOP

		# reset waypoints
		self.b = False
		self.a = False
		self.pose = False
		self.target = False
		self.wpt_tolerance = 0.0
		self.wpt_linear_speed = 0.0
		self.wpt_linear_ramp_down_at_dist = 0.0
		self.wpt_linear_ramp_down_speed = 0.0

		# initialize navigation state vars
		self.dist = 0.0
		self.dist_minimum = 20000000.0
		self.bearing = 0.0
		self.heading_err = 0.0 
		self.heading_err_minimum = self.pi2
		self.ab_len = 0.0
		self.ab_dist_to_pose = 0.0
		self.ab_norm = [0,0]
		self.target_dist = 0.0
		self.target_bearing = 0.0
		self.target_heading_err = 0.0 

		# PID drive controller
		self.pid_drive = pid_controller(self.update_interval)
		self.pid_drive.set_parameters(self.drive_kp, self.drive_ki, self.drive_kd, self.drive_integral_max)

		# PID turn controller
		self.pid_turn = pid_controller(self.update_interval)
		self.pid_turn.set_parameters(self.turn_kp, self.turn_ki, self.turn_kd, self.turn_integral_max)

		# initialize output
		self.linear_speed = 0.0
		self.angular_speed = 0.0

		# debug
		self.debug = debug
		if self.debug:
			self.debug_time_stamp = 0.0
			self.print_count = 0

	# call whenever a new pose is available
	def pose_update (self, easting, northing, yaw):
		self.pose = [easting, northing, yaw]

	# call to set a new navigation destination waypoint
	def navigate (self, destination, origin):
		self.b = destination
		if origin != False:
			self.a = origin
		else:
			self.a = self.pose	

		# set speed and waypoint reached tolerance
		self.wpt_tolerance = float(self.b[WPT_TOLERANCE])
		if self.wpt_tolerance < 0.01:
			self.wpt_tolerance = self.wpt_tolerance_default
		self.wpt_linear_speed = float(self.b[WPT_SPEED])
		if self.wpt_linear_speed < 0.01:
			self.wpt_linear_speed = self.wpt_linear_speed_default

		self.wpt_linear_ramp_down_speed = self.wpt_linear_ramp_down_speed_default
		self.wpt_linear_ramp_down_at_dist = self.wpt_linear_ramp_down_at_dist_default

		# calculate ab-line properties (used by the drive function)
		self.ab_len = sqrt((self.b[WPT_E]-self.a[WPT_E])**2 + (self.b[WPT_N]-self.a[WPT_N])**2) # length of ab line
		self.ab_norm = [(self.b[WPT_E]-self.a[WPT_E])/self.ab_len,(self.b[WPT_N]-self.a[WPT_N])/self.ab_len] # normalized ab vector
		self.a_dot_ab_norm = self.a[0]*self.ab_norm[0] + self.a[1]*self.ab_norm[1] # (vector defined by point a) dot (normalized ab vector) 

		# reset state variables
		self.dist_minimum = 20000000.0
		self.state = self.STATE_DRIVE_INIT
	
	# call to stop navigating to the destination
	def stop (self):
		if self.debug:
			print "Stop"
		self.a = False
		self.b = False
		self.linear_speed = 0.0
		self.angular_speed = 0.0
		self.state = self.STATE_STOP

	# call to temporaily stop navigating to the destination
	def standby (self):
		if self.debug:
			print "Navigation standby"
		self.linear_speed = 0.0
		self.angular_speed = 0.0
		self.prev_state = self.state
		self.state = self.STATE_STANDBY

	# call to resume navigating to the destination
	def resume (self):
		if self.debug:
			print "Resuming navigation"
		self.state = self.prev_state

	# initialize drive towards destination (b) waypoint
	def drive_init(self):
		if self.debug:
			print "Driving towards wpt"
		self.pid_drive.reset ()
		self.heading_err_minimum = self.pi2
		self.state = self.STATE_DRIVE

	# drive towards destination (b) waypoint
	def drive(self):
		if fabs(self.target_heading_err) > self.turn_start_at_heading_err:
			self.state = self.STATE_TURN_INIT
		else:
			self.angular_speed = self.pid_drive.update (self.target_heading_err) # get controller output
			if self.dist > self.wpt_linear_ramp_down_at_dist:
				self.linear_speed = self.wpt_linear_speed
			else:
				self.linear_speed = self.wpt_linear_speed - (1 - self.dist/self.wpt_linear_ramp_down_at_dist)*(self.wpt_linear_speed - self.wpt_linear_ramp_down_speed)

	# initialize turn about own center (not applicable to all robots)
	def turn_init(self):
		if self.debug:		
			print 'Turning to adjust heading error: %.1f' %(self.target_heading_err*self.rad_to_deg)
		self.pid_turn.reset ()
		self.heading_err_minimum = self.pi2
		self.state = self.STATE_TURN

	# turn about own center (not applicable to all robots)
	def turn(self):
		# check if we have turned enough
		if fabs(self.target_heading_err) <= self.turn_acceptable_heading_err:
			self.state = self.STATE_DRIVE_INIT
		else:
			self.angular_speed = self.pid_turn.update (self.target_heading_err) # get controller output
			self.linear_speed = 0.0

	# return true if we have arrived at the destination (b) waypoint
	def arrived_at_waypoint(self):
		arrived = False
		if self.dist <= self.wpt_tolerance: # if we are inside the acceptable tolerance perimeter
			if self.dist > self.dist_minimum: # if we are moving away so we won't get any closer without turning.
				arrived = True
		return arrived

	# calculate distance, bearing, heading error
	def update_navigation_state(self):
		self.dist = sqrt((self.b[0]-self.pose[0])**2 + (self.b[1]-self.pose[1])**2) # dist to destination
		if self.dist < self.dist_minimum: # used for arrival detection
			self.dist_minimum = self.dist
		easting_component = self.b[0] - self.pose[0]  # destination bearing (angle with easting axis)
		northing_component = self.b[1] - self.pose[1]
		self.bearing = self.angle_limit(atan2 (northing_component, easting_component)) 
		self.heading_err = self.angle_diff(self.bearing, self.pose[2]) # signed diff. heading to destination bearing
		if fabs(self.heading_err) < self.heading_err_minimum:
			self.heading_err_minimum = fabs(self.heading_err)

		# find distance to the point on the ab-line that is closest to the robot
		pose_dot_ab_norm = self.pose[0]*self.ab_norm[0] + self.pose[1]*self.ab_norm[1] # (vector defined by pose) dot (normalized ab vector)
		d = pose_dot_ab_norm - self.a_dot_ab_norm  # distance along the ab line 

		pt = [self.a[0] + d*self.ab_norm[0], self.a[1] + d*self.ab_norm[1]] # define closest point along the ab line
		self.ab_dist_to_pose = sqrt((self.pose[WPT_E]-pt[WPT_E])**2 + (self.pose[WPT_N]-pt[WPT_N])**2)
		if d > self.ab_len: # the pose lies beyond the end of the ab line and the closest point is therefore in fact b
			# set b as target point	
			self.target = [self.b[0], self.b[1]]	
		elif d < 0: # the pose lies before the beginning of the ab line so the closest point is in fact a
		#	self.target = [self.a[0], self.a[1]]			
			self.target = [self.b[0], self.b[1]]	
		else: # the closest point is defined by A + d(B-A)
			dist_pt_to_b = sqrt((self.b[WPT_E]-pt[WPT_E])**2 + (self.b[WPT_N]-pt[WPT_N])**2)  # calc distance from closest point to b
			dist_pt_to_target = dist_pt_to_b*self.target_percent # define distance from closest point to target point
			if dist_pt_to_target > self.target_ahead:
				dist_pt_to_target = self.target_ahead
			self.target = [pt[0] + dist_pt_to_target*self.ab_norm[0], pt[1] + dist_pt_to_target*self.ab_norm[1]] # define target point

		# now navigate to the target point...
		self.target_dist = sqrt((self.target[0]-self.pose[0])**2 + (self.target[1]-self.pose[1])**2) # dist to target
		self.target_bearing = self.angle_limit(atan2 (self.target[1]-self.pose[1], self.target[0]-self.pose[0])) #  target bearing (angle with easting axis)
		self.target_heading_err = self.angle_diff(self.target_bearing, self.pose[2]) # signed diff. heading to target bearing

		#print self.target_dist, self.target_heading_err, self.a[0], self.a[1], self.b[0], self.b[1], self.pose[0], self.pose[1], self.target[0], self.target[1] 


	# make sure we don't exceed maximum linear and angular speed
	def limit_speed(self):
		if self.linear_speed > self.linear_speed_max:
			self.linear_speed = self.linear_speed_max
		elif self.linear_speed < -self.linear_speed_max:
			self.linear_speed = -self.linear_speed_max

		if self.angular_speed > self.angular_speed_max:
			self.angular_speed = self.angular_speed_max
		elif self.angular_speed < -self.angular_speed_max:
			self.angular_speed = -self.angular_speed_max

	# waypoint navigation updater, returns status, and linear and angular speed to the controller
	def update(self, time_stamp):
		status = self.UPDATE_NONE
		if self.state != self.STATE_STOP and self.pose != False: # if we have a valid pose

			# calculate distance, bearing, heading error
			self.update_navigation_state()

			# are we there yet?
			if self.arrived_at_waypoint():
				if self.debug:
					print 'Arrived at waypoint %s: ' % self.b[2]
				status = self.UPDATE_ARRIVAL
			
			else:
				# run navigation state machine	
				if self.state == self.STATE_DRIVE_INIT:
					self.drive_init()
				elif self.state == self.STATE_DRIVE:
					self.drive()
				elif self.state == self.STATE_TURN_INIT:
					self.turn_init()
				elif self.state == self.STATE_TURN:
					self.turn()

				self.limit_speed() # don't exceed defined speed limitations

				# print result (debug)
				if self.debug:					
					self.print_count += 1
					if self.print_count % self.print_interval == 0:
						#print "dT: %.3f dist: %.2f bearing: %.1f heading err: %.2f heading err min: %.2f t.dist: %.2f t.bearing: %.1f  linspd: %.3f angspd: %.3f" \
						#	% ((time_stamp-self.debug_time_stamp)/self.print_interval, self.dist, self.bearing*180.0/pi, \
						#	self.heading_err*self.rad_to_deg, self.heading_err_minimum*self.rad_to_deg, self.target_dist, self.target_bearing*180.0/pi, self.linear_speed, self.angular_speed)
						print "state %d dist %6.2f ab %.2f bearing %5.1f heading err %5.1f t.dist %5.2f t.heading_err %5.1f  linspd %.3f angspd %.3f" \
							% (self.state, self.dist, self.ab_dist_to_pose, self.bearing*180.0/pi, \
							self.heading_err*self.rad_to_deg, self.target_dist, self.target_heading_err*self.rad_to_deg, self.linear_speed, self.angular_speed)
						self.debug_time_stamp = time_stamp

		# return result
		return (status, self.linear_speed, self.angular_speed)
	
	# return angle within [0;2pi[
	def angle_limit (self, angle):
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

