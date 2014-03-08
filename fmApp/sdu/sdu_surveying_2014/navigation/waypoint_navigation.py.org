#!/usr/bin/env python
#/****************************************************************************
# Waypoint Navigation
# Copyright (c) 2013, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or withoutn
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
2013-09-22 KJ Added support for initialization of some of the parameters
2013-12-03 KJ Added ramp up which works like the previous ramp down
"""

# imports
from math import pi, atan2, sqrt, fabs
from pid_controller import pid_controller

class waypoint_navigation():
	def __init__(self, update_rate, drive_kp, drive_ki, drive_kd, drive_max_output, turn_kp, turn_ki, turn_kd, turn_max_output, max_linear_vel, max_angular_vel, wpt_tol_default, wpt_def_drive_vel, wpt_def_turn_vel, target_distance, turn_start_at_heading_err, turn_stop_at_heading_err, ramp_drive_vel_at_dist, ramp_min_drive_vel, ramp_turn_vel_at_angle, ramp_min_turn_vel, debug):

		# constants
		self.UPDATE_NONE = 0
		self.UPDATE_ARRIVAL = 1
		self.pi2 = 2.0*pi
		self.deg_to_rad = pi/180.0
		self.rad_to_deg = 180.0/pi

		# list index for destination (b) and origin (a) waypoints
		self.W_E = 0 
		self.W_N = 1
		self.W_YAW = 2
		self.W_ID = 3
		self.W_MODE = 4
		self.W_TOL = 5
		self.W_LIN_VEL = 6
		self.W_ANG_VEL = 7
		self.W_WAIT = 8
		self.W_IMPLEMENT = 9

		# parameters
		self.update_rate = update_rate # [Hz]
		self.update_interval = (1.0/self.update_rate) # [s]
		self.max_linear_vel = max_linear_vel # [m/s]
		self.max_angular_vel = max_angular_vel # [radians/s]
		self.angular_vel_limit = self.max_angular_vel
		self.drive_kp = drive_kp
		self.drive_ki = drive_ki
		self.drive_kd = drive_kd
		self.drive_max_output = drive_max_output
		self.turn_kp = turn_kp
		self.turn_ki = turn_ki
		self.turn_kd = turn_kd
		self.turn_max_output = turn_max_output
		self.wpt_tolerance_default = wpt_tol_default # [m]
		self.wpt_def_drive_vel = wpt_def_drive_vel # [m/s]
		self.wpt_def_turn_vel = wpt_def_turn_vel # [m/s]
		self.target_ahead = target_distance # [m] the intermediate target is along the ab line 'self.target_ahead' meters ahead of the pose 
		self.target_percent = 1.0 # [0;1] when distance to b is less than self.target_ahead, the target is self.target_percent times the distance to b ahead of the pose
		self.turn_start_at_heading_err = turn_start_at_heading_err*self.deg_to_rad # [radians] set to 2pi if not applicable to the robot9
		self.turn_acceptable_heading_err = turn_stop_at_heading_err*self.deg_to_rad # [radians]
		self.ramp_drive_vel_at_dist_default = ramp_drive_vel_at_dist # [m]
		self.ramp_min_drive_vel_default = ramp_min_drive_vel # [m/s]
		self.ramp_turn_vel_at_angle_default = ramp_turn_vel_at_angle*pi/180.0 # [deg]
		self.ramp_min_turn_vel_default = ramp_min_turn_vel # [rad/s]

		self.print_interval = self.update_rate/20

		# navigation controller state machine
		self.STATE_STOP = 0
		self.STATE_STANDBY = 1
		self.STATE_DRIVE = 2
		self.STATE_TURN = 3

		self.state = self.STATE_STOP
		self.prev_state = self.STATE_STOP

		# reset waypoints
		self.start = False
		self.a = False
		self.b = False
		self.pose = False
		self.target = False
		self.wpt_tolerance = 0.0
		self.wpt_drive_vel = 0.0
		self.wpt_turn_vel = 0.0
		self.wpt_ramp_drive_vel_at_dist = 0.0
		self.wpt_ramp_min_drive_vel = 0.0

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
		self.target_heading_err_prev = 0.0 
		self.turn_bearing_origin = 0.0

		# PID drive controller
		self.pid_drive = pid_controller(self.update_interval)
		self.pid_drive.set_parameters(self.drive_kp, self.drive_ki, self.drive_kd, self.drive_max_output)

		# PID turn controller
		self.pid_turn = pid_controller(self.update_interval)
		self.pid_turn.set_parameters(self.turn_kp, self.turn_ki, self.turn_kd, self.turn_max_output)

		# initialize output
		self.linear_vel = 0.0
		self.angular_vel = 0.0

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
		self.b = destination # where we are going

		if origin != False:
			self.a = origin
		else:
			self.a = self.pose	

		self.start = self.pose # where we started driving from (updated in drive_init())

		# set velocity and waypoint reached tolerance
		self.wpt_tolerance = float(self.b[self.W_TOL])
		if self.wpt_tolerance < 0.001:
			self.wpt_tolerance = self.wpt_tolerance_default

		# set drive velocity
		self.wpt_drive_vel = float(self.b[self.W_LIN_VEL])
		if self.wpt_drive_vel < 0.001:
			self.wpt_drive_vel = self.wpt_def_drive_vel

		self.wpt_ramp_min_drive_vel = self.ramp_min_drive_vel_default
		self.wpt_ramp_drive_vel_at_dist = self.ramp_drive_vel_at_dist_default

		# set turn velocity limit
		if self.b[self.W_ANG_VEL] > 0.001:
			self.turn_vel_limit = self.b[self.W_ANG_VEL]
		else:
			self.turn_vel_limit = self.wpt_def_turn_vel

		if self.turn_vel_limit > self.max_angular_vel:
				self.turn_vel_limit = self.max_angular_vel	

		# calculate ab-line properties (used by the drive function)
		self.ab_len = sqrt((self.b[self.W_E]-self.a[self.W_E])**2 + (self.b[self.W_N]-self.a[self.W_N])**2) # length of ab line
		self.ab_norm = [(self.b[self.W_E]-self.a[self.W_E])/self.ab_len,(self.b[self.W_N]-self.a[self.W_N])/self.ab_len] # normalized ab vector
		self.a_dot_ab_norm = self.a[0]*self.ab_norm[0] + self.a[1]*self.ab_norm[1] # (vector defined by point a) dot (normalized ab vector) 

		# reset state variables
		self.dist_minimum = 20000000.0

		self.update_navigation_state()
		self.drive_init()
	
	# call to stop navigating to the destination
	def stop (self):
		if self.debug:
			print "Stop"
		self.a = False
		self.b = False
		self.linear_vel = 0.0
		self.angular_vel = 0.0
		self.state = self.STATE_STOP

	# call to temporaily stop navigating to the destination
	def standby (self):
		if self.debug:
			print "Navigation standby"
		self.linear_vel = 0.0
		self.angular_vel = 0.0
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
		self.start = self.pose # where we started driving from
		self.pid_drive.reset ()
		self.heading_err_minimum = self.pi2
		self.state = self.STATE_DRIVE
		self.drive() # execute the update immediately

	# drive towards destination (b) waypoint
	def drive(self):
		if fabs(self.target_heading_err) > self.turn_start_at_heading_err and self.dist > self.wpt_tolerance_default:
			self.turn_init()
		else:
			ramp = 1.0
			if self.dist < self.wpt_ramp_drive_vel_at_dist: # close to destination
				ramp = self.dist/self.wpt_ramp_drive_vel_at_dist
			elif self.dist_start < self.wpt_ramp_drive_vel_at_dist: # close to starting point
				ramp = self.dist_start/self.wpt_ramp_drive_vel_at_dist

			self.linear_vel = self.wpt_drive_vel - (1-ramp)*(self.wpt_drive_vel - self.wpt_ramp_min_drive_vel)

			#if self.dist > self.wpt_tolerance_default:
			self.angular_vel = ramp*self.pid_drive.update (self.target_heading_err) # get controller output

	# initialize turn about own center (not applicable to all robots)
	def turn_init(self):
		if self.debug:		
			print 'Turning to adjust heading error: %.1f' %(self.target_heading_err*self.rad_to_deg)
		self.pid_turn.reset ()
		self.turn_bearing_origin = self.pose[2]
		self.heading_err_minimum = self.pi2
		self.state = self.STATE_TURN
		self.turn() # execute the update immediately

	# turn about own center (not applicable to all robots)
	def turn(self):
		# check if we have turned enough
		angular_dist_to_target = fabs(self.target_heading_err)
		if angular_dist_to_target <= self.turn_acceptable_heading_err:
			self.drive_init()
		else:
			self.angular_vel = self.pid_turn.update (self.target_heading_err) # get controller output

			angular_dist_from_origin = fabs(self.angle_diff (self.pose[2], self.turn_bearing_origin))
			actual_limit = self.turn_vel_limit
			if angular_dist_to_target < self.ramp_turn_vel_at_angle_default:
				actual_limit = self.turn_vel_limit - (1 - angular_dist_to_target/self.ramp_turn_vel_at_angle_default)*(self.turn_vel_limit - self.ramp_min_turn_vel_default)
	
			elif angular_dist_from_origin < self.ramp_turn_vel_at_angle_default:
				actual_limit = self.turn_vel_limit - (1 - angular_dist_from_origin/self.ramp_turn_vel_at_angle_default)*(self.turn_vel_limit - self.ramp_min_turn_vel_default)
			if self.angular_vel > actual_limit:
				self.angular_vel = actual_limit
			elif self.angular_vel < -actual_limit:
				self.angular_vel = -actual_limit

			self.linear_vel = 0.0

	# return true if we have arrived at the destination (b) waypoint
	def arrived_at_waypoint(self):
		arrived = False
		if self.dist <= self.wpt_tolerance: # if we are inside the acceptable tolerance perimeter
			if self.dist < 0.01: # if we are VERY close.
				arrived = True
			elif self.dist > self.dist_minimum: # if we are moving away so we won't get any closer without turning.
				arrived = True
			elif fabs(self.angle_diff (self.target_heading_err, self.target_heading_err_prev)) > pi/8.0: # if large bearing jump
				arrived = True
		return arrived

	# calculate distance, bearing, heading error
	def update_navigation_state(self):
		self.dist = sqrt((self.b[self.W_E]-self.pose[0])**2 + (self.b[self.W_N]-self.pose[1])**2) # dist to destination
		self.dist_start = sqrt((self.start[self.W_E]-self.pose[0])**2 + (self.start[self.W_N]-self.pose[1])**2) # dist to starting position
		if self.dist < self.dist_minimum: # used for arrival detection
			self.dist_minimum = self.dist
		easting_component = self.b[self.W_E] - self.pose[0]  # destination bearing (angle with easting axis)
		northing_component = self.b[self.W_N] - self.pose[1]
		self.bearing = self.angle_limit(atan2 (northing_component, easting_component)) 
		self.heading_err = self.angle_diff(self.bearing, self.pose[2]) # signed diff. heading to destination bearing
		if fabs(self.heading_err) < self.heading_err_minimum:
			self.heading_err_minimum = fabs(self.heading_err)

		# find distance to the point on the ab-line that is closest to the robot
		pose_dot_ab_norm = self.pose[0]*self.ab_norm[0] + self.pose[1]*self.ab_norm[1] # (vector defined by pose) dot (normalized ab vector)
		d = pose_dot_ab_norm - self.a_dot_ab_norm  # distance along the ab line 

		pt = [self.a[0] + d*self.ab_norm[0], self.a[1] + d*self.ab_norm[1]] # define closest point along the ab line
		self.ab_dist_to_pose = sqrt((self.pose[self.W_E]-pt[self.W_E])**2 + (self.pose[self.W_N]-pt[self.W_N])**2)
		if d > self.ab_len: # the pose lies beyond the end of the ab line and the closest point is therefore in fact b
			# set b as target point	
			self.target = [self.b[self.W_E], self.b[self.W_N]]	
		elif d < 0: # the pose lies before the beginning of the ab line so the closest point is in fact a
		#	self.target = [self.a[0], self.a[1]]			
			self.target = [self.b[self.W_E], self.b[self.W_N]]	
		else: # the closest point is defined by A + d(B-A)
			dist_pt_to_b = sqrt((self.b[self.W_E]-pt[self.W_E])**2 + (self.b[self.W_N]-pt[self.W_N])**2)  # calc distance from closest point to b
			dist_pt_to_target = dist_pt_to_b*self.target_percent # define distance from closest point to target point
			if dist_pt_to_target > self.target_ahead:
				dist_pt_to_target = self.target_ahead
			self.target = [pt[0] + dist_pt_to_target*self.ab_norm[0], pt[1] + dist_pt_to_target*self.ab_norm[1]] # define target point

		# now navigate to the target point...
		self.target_dist = sqrt((self.target[0]-self.pose[0])**2 + (self.target[1]-self.pose[1])**2) # dist to target
		self.target_bearing = self.angle_limit(atan2 (self.target[1]-self.pose[1], self.target[0]-self.pose[0])) #  target bearing (angle with easting axis)
		self.target_heading_err_prev = self.target_heading_err
		self.target_heading_err = self.angle_diff(self.target_bearing, self.pose[2]) # signed diff. heading to target bearing

		#print self.target_dist, self.target_heading_err, self.a[0], self.a[1], self.b[self.W_E], self.b[self.W_N], self.pose[0], self.pose[1], self.target[0], self.target[1] 


	# make sure we don't exceed maximum linear and angular velocity
	def limit_vel(self):
		if self.linear_vel > self.max_linear_vel:
			self.linear_vel = self.max_linear_vel
		elif self.linear_vel < -self.max_linear_vel:
			self.linear_vel = -self.max_linear_vel

		# perform angular velocity limit
		if self.angular_vel > self.angular_vel_limit:
			self.angular_vel = self.angular_vel_limit
		elif self.angular_vel < -self.angular_vel_limit:
			self.angular_vel = -self.angular_vel_limit

	# waypoint navigation updater, returns status, and linear and angular velocity to the controller
	def update(self, time_stamp):
		status = self.UPDATE_NONE
		if self.state != self.STATE_STOP and self.pose != False: # if we have a valid pose

			# calculate distance, bearing, heading error
			self.update_navigation_state()

			# are we there yet?
			if self.arrived_at_waypoint():
				status = self.UPDATE_ARRIVAL
			
			else:
				# print result (debug)
				if self.debug:					
					self.print_count += 1
					if self.print_count % self.print_interval == 0:
						print "%.3f  state %d dist %6.2f bearing %5.1f  t_dist %5.2f t_head_err %5.1f ab_dist %.2f  lin_v %5.2f ang_v %5.2f" % (time_stamp, self.state, self.dist, self.bearing*self.rad_to_deg, self.target_dist, self.target_heading_err*self.rad_to_deg, self.ab_dist_to_pose, self.linear_vel, self.angular_vel)
						self.debug_time_stamp = time_stamp

				# run navigation state machine	
				if self.state == self.STATE_DRIVE:
					self.drive()
				elif self.state == self.STATE_TURN:
					self.turn()

				self.limit_vel() # don't exceed defined velocity limitations

		# return result
		return (status, self.linear_vel, self.angular_vel)
	
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

