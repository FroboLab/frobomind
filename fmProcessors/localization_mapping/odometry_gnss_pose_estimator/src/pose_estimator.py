#!/usr/bin/env python
#*****************************************************************************
# Pose Estimator Library
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
This file contains a pose estimator library designed for field robots.

A pose defines the position and orientation of a shape in world space
(or the parent coordinate space). In this implementation the pose describes
the robot lateral position in a geographical coordinate plane.

In this library the ENU coordinate system is used as reference to allow use
of GNSS based coordinates represented in a Transverse Mercator projection: 

	Pose position vector: [easting, northing]
	Pose orientation vector: [yaw]
		Positive rotation counter clockwise, origo at easting axis 

The pose is estimated based on sensor inputs from:
	Robot odometry feedback:
		Wheel encoders (typically fused with a gyro)
	Absolute position sensor:
		Real Time Kinematic GPS (RTK-GPS)

Each sensor input is validated and filtered, and the actual variance is
estimated. This is based on generic and parameterized knowledge about
the field robot and sensor types.

An Extended Kalman Filter (EKF) then uses the robot odometry feedback as
system input (prediction) and the GNSS and AHRS information as measurement
inputs (correction).

The library is expected to be ported to C++ for performance optimations
but during development and test everything will be written in Python.

Revision
2013-04-25 KJ First version
2014-03-18 KJ Various bug fixes and computation optimizations
2014-04-23 KJ Fixed EKF problems and updated preprocessor parameters.
"""
# imports
import numpy as np
from math import sqrt, pi, sin, cos, atan2, fabs
from numpy import matrix, array, linalg, mat

class odometry_gnss_pose_preprocessor():
	def __init__(self, max_speed):
		self.deg_to_rad = pi/180.0
		self.rad_to_deg = 180.0/pi
		self.pi2 = 2.0*pi
		self.max_speed = max_speed
		self.buffer_period = 2.0 # [s] PARAMETER
		self.buf_init_size = 25
		self.gnss = []
		self.gnss_interval = 0.0 # [s]
		self.gnss_interval_valid = False
		self.gnss_buf_size = self.buf_init_size
		self.imu = []
		self.imu_interval = 0.0 # [s]
		self.imu_interval_valid = False
		self.imu_buf_size = self.buf_init_size
		self.odo = []
		self.odo_interval = 0.0 # [s]
		self.odo_interval_valid = False
		self.odo_buf_size = self.buf_init_size
		self.gnss_heading_min_dist = 0.25 # [m]
		self.orientation_odo_gnss_dist_max_diff_percent = 10.0
		self.orientation_odo_max_angle = 5.0 * self.deg_to_rad
		self.gnss_orientation_prev_yaw = -10.0
		self.gnss_std_dev_max = 20000000.0
		self.gnss_latest_invalid = 0.0

	def gnss_estimate_heading (self):
		valid = False
		heading = 0.0
		dist = 0.0
		var_heading = pi
		E = 1
		N = 2
		if self.gnss_interval_valid and len(self.gnss) == self.gnss_buf_size: # if we have a full gnss buffer
			if self.gnss[-1][3] == 4 and self.gnss[0][3] == 4: # if we have a RTK fixed solution
				dist = sqrt ((self.gnss[-1][E]- self.gnss[0][E])**2 + (self.gnss[-1][N]- self.gnss[0][N])**2)
				if dist >= self.gnss_heading_min_dist: # if we have travelled a configurable minimum distance 
					# calc gnss coordinate vector heading
					e_vec_len = self.gnss[-1][E] - self.gnss[0][E]
					n_vec_len = self.gnss[-1][N] - self.gnss[0][N]
					heading = self.angle_limit(atan2 (n_vec_len, e_vec_len))
					var_heading = 0.0 # OI OI MUST BE CALCULATED !!!			
					valid = True
		return (valid, heading, var_heading, dist)
	
	def estimate_absolute_orientation (self):
		valid = False
		yaw = 0.0
		# estimate heading based on gnss waypoints
		(heading_valid, heading, var_heading, gnss_dist) = self.gnss_estimate_heading()
		if heading_valid == True:
			(odo_valid, odo_dist, odo_angle) = self.odometry_buf_distance_angle() # calc traversed dist and angle in odo sliding window buffer
			if odo_valid == True:
				if fabs (odo_angle) <= self.orientation_odo_max_angle:
					if fabs(gnss_dist-odo_dist) < self.orientation_odo_gnss_dist_max_diff_percent/100.0*odo_dist:
						yaw = heading
						valid = True
		return (valid, self.angle_limit(yaw))

	def gnss_estimate_variance_pos(self):
		std_dev = self.gnss_std_dev_max
		if len(self.gnss) > 0:
			if self.gnss[-1][3] == 4: # rtk fixed solution
				std_dev = 0.02 * self.gnss[-1][5]
			elif self.gnss[-1][3] == 5: # rtk float solution
				std_dev = 2.0
				std_dev = 2.0 * self.gnss[-1][5]
			elif self.gnss[-1][3] == 2: # dgps solution
				std_dev = 5.0
				std_dev = 5.0 * self.gnss[-1][5]
			elif self.gnss[-1][3] == 1: # sps solution
				std_dev = 15.0 * self.gnss[-1][5]
		return std_dev**2

	def gnss_validate_new_position(self, time_stamp, easting, northing):
		valid = 1
		if self.gnss != []: # if we have previous positions to validate against
			# check for sudden position jumps
			dtime = time_stamp - self.gnss[-1][0]
			ddist = sqrt((easting - self.gnss[-1][1])**2 + (northing - self.gnss[-1][2])**2)
			max_dist = self.max_speed *dtime
			if ddist > 2*max_dist: # if distance larger than 2 * theoretical maximum distance
				valid = 0
				#print "  GNSS unexpected position jump %.1f m at time stamp: %.3f: E%.3f, N%.3f" % (ddist, time_stamp, easting, northing)
		return valid

	def gnss_new_data (self, time_stamp, easting, northing, solution, sat, hdop):
		valid = False
		if solution > 0: # if satellite fix
			if self.gnss_validate_new_position (time_stamp, easting, northing): 
				self.gnss.append([time_stamp, easting, northing, solution, sat, hdop]) # add measurement to sliding window
				valid = True
				if self.gnss_interval_valid == False and len(self.gnss) == self.buf_init_size: # if time to calc update interval
					self.gnss_interval_valid = True
					self.gnss_interval = (self.gnss[-1][0] - self.gnss[0][0])/(self.buf_init_size - 1.0)
					print "  Estimated GNSS measurement interval: %.3fs" % (self.gnss_interval)
					self.gnss_buf_size = int(self.buffer_period / self.gnss_interval)
				while len(self.gnss) > self.gnss_buf_size: # trim buffer length to size
					self.gnss.pop(0)	
		if valid == False: # dischard the entire buffer
			self.gnss = []
			self.gnss_latest_invalid = time_stamp
		return valid

	def imu_new_data(self, time_stamp, yaw_rate):
		self.imu.append([time_stamp, yaw_rate])
		if self.imu_interval_valid == False and len(self.imu) == self.buf_init_size: # if time to calc update interval
			self.imu_interval_valid = True
			self.imu_interval = (self.imu[-1][0] - self.imu[0][0])/(self.buf_init_size - 1.0)
			print "  Estimated IMU measurement interval: %.3fs" % (self.imu_interval)
			self.imu_buf_size = int(self.buffer_period / self.imu_interval)
		while len(self.imu) > self.imu_buf_size:
			self.imu.pop(0)

	def odometry_new_data(self, time_stamp, delta_dist, delta_angle, forward):
		self.odo.append([time_stamp, delta_dist, delta_angle, forward])
		if self.odo_interval_valid == False and len(self.odo) == self.buf_init_size: # update buffer size based on gnss update interval
			self.odo_interval_valid = True
			self.odo_interval = (self.odo[-1][0] - self.odo[0][0])/(self.buf_init_size - 1.0)
			print "  Estimated odometry update interval: %.3fs" % (self.odo_interval)
			self.odo_buf_size = int(self.buffer_period / self.odo_interval)
		while len(self.odo) > self.odo_buf_size:
			self.odo.pop(0)

	def odometry_buf_distance_angle (self):
		valid = False
		buffer_dist = 0.0
		buffer_angle = 0.0
		if  len(self.odo) == self.odo_buf_size: # if we have a full odometry buffer
			valid = True
			for i in xrange(self.odo_buf_size):
				buffer_dist += self.odo[i][1]
				buffer_angle += self.odo[i][2]
				if self.odo[i][3] == False: # if we have been driving backwards the data may be invalid
					valid = False
		return (valid, buffer_dist, buffer_angle)

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

class odometry_pose_ekf():
	def __init__(self):
		self.pi2 = 2.0*pi
		self.Q = np.zeros((3,3))
		self.H = np.matrix([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]]) # Observation matrix
		# maximum variance for the initial guess (half the Earth circumference and half the unit circle). 
		self.initial_guess ([0.0, 0.0, 0.0], 20000000.0**2, pi**2)

	def initial_guess (self, pose, var_pos, var_yaw):
		self.X = np.matrix(pose)
		self.P = np.matrix([[var_pos,0.0,0.0],[0.0,var_pos,0.0], [0.0,0.0,var_yaw]]) 

	def system_update (self, delta_dist, var_dist, delta_angle, var_angle):
		u = [delta_dist, delta_angle]

		# predicted (A priori) state estimate: X-[t] = X[t-1] + u[t]
		priX = self.f(self.X, u)

		# linearize the system model around X
		F = self.F(self.X, u)

		# predicted (A priori) error covariance estimate: P-[t] = P[t-1] + Q
		self.Q = self.Q + np.matrix([[var_dist, 0.0, 0.0],[0.0, var_dist, 0.0],[0.0, 0.0, var_angle]])
		priP = F*self.P*F.T + self.Q 

		# housekeeping
		self.X = priX # advance to next update step
		self.P = priP
		return array(self.X)[0]

	def measurement_update (self, pose, var_pos, var_yaw):
		# Compute Kalman gain: K[t] = P-[t]/(P-[t] + R)
		R = np.matrix([[var_pos, 0., 0.],[0., var_pos, 0.],[0., 0., var_yaw]])
		K = self.P*self.H.T*linalg.inv(self.H*self.P*self.H.T + R)

		# updated (A posteriori) estimate with measurement z[t]: X[t] = X-[t] + K[t]*(z[t] - X-[t])
		mr = np.matrix(pose).T - self.X.T # measurement residual (z[t] - X-[t])
		postX = self.X + (K*mr).T # updated state estimate

		# updated (A posteriori) error covariance: P[t] = (1 - K[t])*P-[t]
		postP = (np.identity(3)-K*self.H)*self.P

		# housekeeping
		self.Q[0,0] = 0.0 # reset the position part of the process noise covariance matrix
		self.Q[1,1] = 0.0
		if pose[2] != array(self.X)[0][2]: # if the yaw angle was actually updated
			self.Q[2,2] = 0.0
		self.X = postX # advance to next update step
		self.P = postP # save A posteriori error variance estimate
		return array(self.X)[0]

	def f (self, X, u): # predict state
		# X is the state vector at timestep k: [x, y, theta].T
		# u is the system input vector at timestep k: [delta_dist, delta_angle].T
		X = array(X)[0]
		theta = X[2]
		dTheta = u[1]
		return np.matrix([X[0]+u[0]*cos(theta+dTheta/2.0),X[1]+u[0]*sin(theta+dTheta/2.0),theta+dTheta])

	def F (self, X, u): # Linear approximation of the system model around X (Jacobian matrix)
		# X is the state vector at timestep k: [x, y, theta].T
		# u is the system input vector at timestep k: [delta_dist, delta_angle].T
		X = array(X)[0]
		theta = X[2]
		dTheta = u[1]
		return (np.matrix([[1., 0., -u[0]*sin(theta+dTheta/2.)], \
			[0., 1., u[0]*cos(theta+dTheta/2.)], \
			[0., 0., 1.]]))

