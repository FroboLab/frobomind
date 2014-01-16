#!/usr/bin/env python
#/****************************************************************************
# Survey application script
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
This file logs all details regarding the robot pose data while the robot is
waiting at a waypoint.

Revision
2013-12-09 KJ First version
"""

import rospy
from msgs.msg import waypoint_navigation_status, gpgga_tranmerc
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import TwistStamped
from math import pi, asin, atan2, sqrt

import numpy as np

class linear_least_squares_2d_model:
	def __init__(self):
		self.poly = False

	def fit(self, points):
		pts_arr = np.array(points)
		z = np.polyfit (pts_arr[:,0], pts_arr[:,1], 1)
		self.poly = np.poly1d(z)
		return z

class log_node():
	def __init__(self):
		# constants
		self.deg_to_rad = pi/180.0
		self.rad_to_deg = 180.0/pi

		# static parameters
		self.debug = True
		self.update_rate = 20 # [Hz]
		self.wait_before_log = 1.0 # [s]
		self.wait_timeout = 0.0
		self.log_file = 'survey.txt'

		# robot state
		self.STATE_IDLE = 0
		self.STATE_WAIT = 1 # wait until standing still
		self.STATE_LOG = 2
		self.state = self.STATE_IDLE
		self.wptnav_wait = False

		# read parameters

		# get topic names
		cmd_vel_topic = rospy.get_param("~cmd_vel_sub", "/fmCommand/cmd_vel")
		wptnav_status_topic = rospy.get_param("~wptnav_status_sub", "/fmData/wptnav_status")
		imu_topic = rospy.get_param("~imu_sub", "/fmInformation/imu")
		gnss_topic = rospy.get_param("~gpgga_tranmerc_sub", "/fmInformation/gpgga_tranmerc")
		laser_topic = rospy.get_param("~lidar_sub",'/fmSensors/laser_msg')

		# init variables
		self.vel_lin = 1.0
		self.vel_ang = 1.0
		self.b = [0.0, 0.0]
		self.init_log ('#seconds,easting,northing,altitude,dist_to_wpt,pitch,roll,yaw\n')

		# lidar
		self.laser = []
		self.lsq = linear_least_squares_2d_model()

		# setup subscription topic callbacks
		rospy.Subscriber(cmd_vel_topic, TwistStamped, self.on_cmd_vel_topic)
		rospy.Subscriber(wptnav_status_topic, waypoint_navigation_status, self.on_wptnav_status_topic)
		rospy.Subscriber(imu_topic, Imu, self.on_imu_topic)
		rospy.Subscriber(gnss_topic, gpgga_tranmerc, self.on_gnss_topic)
		rospy.Subscriber(laser_topic, LaserScan, self.on_laser_topic)

		# call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def on_cmd_vel_topic(self, msg):
		self.vel_lin = msg.twist.linear.x
		self.vel_ang = msg.twist.angular.z
		if self.state == self.STATE_WAIT or self.state == self.STATE_LOG:
			if self.vel_lin != 0 or self.vel_ang != 0:
				self.end_log()
				if self.debug:
					print '%.3f Survey stop due to velocity command: lin %.2f angular %.2f' % (rospy.get_time(), self.vel_lin, self.vel_ang)

	def on_wptnav_status_topic(self, msg):
		self.wptnav_wait = (msg.state == 2)
		msg_b = [msg.b_easting, msg.b_northing]

		if msg_b != self.b and self.wptnav_wait == True:
			self.b = msg_b

		if self.wptnav_wait == True and self.state == self.STATE_IDLE and self.vel_lin == 0 and self.vel_ang == 0:
			self.begin_wait()
		elif (self.wptnav_wait == False or self.vel_lin != 0 or self.vel_ang != 0) and self.state == self.STATE_LOG:
			self.end_log()				

	def on_imu_topic(self, msg):
		if self.state == self.STATE_LOG:
			# extract yaw, pitch and roll from the quaternion
			qx = msg.orientation.x
			qy = msg.orientation.y
			qz = msg.orientation.z
			qw = msg.orientation.w
			sqx = qx**2
			sqy = qy**2
			sqz = qz**2
			sqw = qw**2
			yaw = atan2(2*(qx*qy + qw*qz), sqw + sqx - sqy - sqz)
			pitch = asin((2*qy*qw) - (2*qx*qz))
			roll = atan2((2*qy*qz) + (2*qx*qw), sqw + sqz - sqy - sqx)
			self.imu.append([pitch, roll])

	def on_gnss_topic(self, msg):
		if self.state == self.STATE_LOG:
			self.gnss.append([msg.easting, msg.northing, msg.alt])

	def on_laser_topic(self, msg):
		self.laser = []
		theta = msg.angle_min
		for i in xrange(len(msg.ranges)):
			r = msg.ranges[i]
			self.laser.append([r*sin(theta),r*cos(theta)]) # polar to cartesian coordinates
			theta += msg.angle_increment
		coeff = self.lsq.fit(self.laser) # determine coefficients for approximated line
		distance = coeff[1] # x=0 is supposed to be right under the antenna

	def begin_wait (self):
		self.wait_timeout = rospy.get_time() + self.wait_before_log
		if self.debug:
			print '%.3f Survey wait until %.3f' % (rospy.get_time(),  self.wait_timeout)
		self.state = self.STATE_WAIT
	
	def begin_log (self):
		self.gnss = []
		self.imu =[]
		self.laser = [] 
		if self.debug:
			print '%.3f Survey begin' % rospy.get_time()
		self.state = self.STATE_LOG

	def init_log (self, header):
		f = open(self.log_file, 'w')
		f.write (header)
		f.close()

	def save_log (self, line):
		f = open (self.log_file, 'a')
		f.write (line)
		f.close()

	def average_gnss_pose(self):
		n = len(self.gnss)
		easting = 0.0
		northing = 0.0
		alt = 0.0
		for i in xrange(n):
			easting += self.gnss[i][0]
			northing += self.gnss [i][1]
			alt += self.gnss [i][2]
		easting /= n
		northing /= n
		alt /= n
		if self.debug:
			print 'GNSS Easting: %.3f Northing: %.3f Altitude: %.3f' % (easting, northing, alt)
		return (easting, northing, alt)

	def average_imu(self):
		n = len(self.imu)
		pitch = 0.0
		roll = 0.0
		for i in xrange(n):
			pitch += self.imu[i][0]
			roll += self.imu[i][1]
		pitch /= n
		roll /= n
		if self.debug:
			print 'IMU Pitch: %.3f Roll: %.3f' % (pitch*self.rad_to_deg, roll*self.rad_to_deg)
		return (pitch, roll)

	def end_log (self):
		(gnss_e, gnss_n, gnss_alt) = self.average_gnss_pose()
		dist_from_wpt =  self.calc_2d_dist ([gnss_e, gnss_n], self.b)
		(imu_pitch, imu_roll) = self.average_imu()
		if self.debug:
			print 'Distance from waypoint: %.3f' % dist_from_wpt

		s = '%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f\n' %(rospy.get_time(), gnss_e, gnss_n, gnss_alt, dist_from_wpt, imu_pitch*self.rad_to_deg, imu_roll*self.rad_to_deg)
		self.save_log(s)

		self.state = self.STATE_IDLE
		if self.debug:
			print '%.3f Survey end' %  rospy.get_time()

	def updater(self):
		while not rospy.is_shutdown():
			if self.state == self.STATE_WAIT:
				if rospy.get_time() >= self.wait_timeout:
					self.begin_log()

			self.r.sleep() # go back to sleep

	def calc_2d_dist (self, a, b):
		return sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

# main function.    
if __name__ == '__main__':
    # initialize the node and name it.
    rospy.init_node('survey_log')

    # go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = log_node()
    except rospy.ROSInterruptException: pass


