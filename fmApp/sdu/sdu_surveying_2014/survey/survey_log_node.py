#!/usr/bin/env python
#/****************************************************************************
# Survey application script
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
#****************************************************************************/
"""
This file logs all details regarding the robot pose data while the robot is
waiting at a waypoint.

Revision
2013-12-09 KJ First version
"""

import rospy
from msgs.msg import waypoint_navigation_status, gpgga_tranmerc
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import TwistStamped
from math import pi, asin, atan2, sqrt
import time
import datetime
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
		self.debug = False
		self.update_rate = 20 # [Hz]
		self.wait_before_log = 2.0 # [s]
		self.wait_timeout = 0.0
		ts = time.time()
		self.log_file = datetime.datetime.fromtimestamp(ts).strftime('survey_%Y-%m-%d-%H-%M-%S.txt')

		# robot state
		self.STATE_IDLE = 0
		self.STATE_WAIT = 1 # wait until standing still
		self.STATE_LOG = 2
		self.state = self.STATE_IDLE

		# read parameters

		# get topic names
		cmd_vel_topic = rospy.get_param("~cmd_vel_sub", "/fmCommand/cmd_vel")
		wptnav_status_topic = rospy.get_param("~wptnav_status_sub", "/fmInformation/wptnav_status")
		pose_topic = rospy.get_param("~pose_sub", "/fmKnowledge/pose")
		imu_topic = rospy.get_param("~imu_sub", "/fmInformation/imu")
		gnss_topic = rospy.get_param("~gpgga_tranmerc_sub", "/fmInformation/gpgga_tranmerc")
		laser_topic = rospy.get_param("~lidar_sub",'/fmSensors/laser_msg')

		# init variables
		self.cmd_vel_lin = 1.0
		self.cmd_vel_ang = 1.0
		self.b = [0.0, 0.0]
		self.b_id = ''
		self.robot_yaw = 0.0
		self.init_log ('#wpt_id\twpt_e\twpt_n\ttime\taverage_time\teasting\tnorthing\taltitude\tdist_to_wpt\tpitch\troll\n')

		# lidar
		self.laser = []
		self.lsq = linear_least_squares_2d_model()

		# setup subscription topic callbacks
		rospy.Subscriber(cmd_vel_topic, TwistStamped, self.on_cmd_vel_topic)
		rospy.Subscriber(wptnav_status_topic, waypoint_navigation_status, self.on_wptnav_status_topic)
		rospy.Subscriber(pose_topic, Odometry, self.on_pose_topic)
		rospy.Subscriber(imu_topic, Imu, self.on_imu_topic)
		rospy.Subscriber(gnss_topic, gpgga_tranmerc, self.on_gnss_topic)
		rospy.Subscriber(laser_topic, LaserScan, self.on_laser_topic)

		# call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def on_cmd_vel_topic(self, msg):
		if self.state == self.STATE_LOG:
			self.cmd_vel_lin = msg.twist.linear.x
			self.cmd_vel_ang = msg.twist.angular.z

			if self.cmd_vel_lin != 0 or self.cmd_vel_ang != 0:
				self.end_log()
				if self.debug:
					print '%.3f Survey stop due to velocity command: lin %.2f angular %.2f' % (rospy.get_time(), self.cmd_vel_lin, self.cmd_vel_ang)

	def on_wptnav_status_topic(self, msg):
		msg_b = [msg.b_easting, msg.b_northing]
		if msg_b != self.b and self.state != self.STATE_LOG:
			self.b = msg_b
			self.b_id = msg.b_id

		wptnav_wait = (msg.state == 2)
		if wptnav_wait == True and self.state == self.STATE_IDLE:
			self.begin_wait()
		elif wptnav_wait == False and self.state == self.STATE_LOG:
			self.end_log()				

	def on_imu_topic(self, msg):
		if self.state == self.STATE_LOG:
			# save accelerometers
			ax = msg.linear_acceleration.x
			ay = msg.linear_acceleration.y
			az = msg.linear_acceleration.z

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

			self.imu.append([yaw, pitch, roll, ax, ay, az])

	def on_pose_topic(self, msg):
		# extract yaw, pitch and roll from the quaternion
		qx = msg.pose.pose.orientation.x
		qy = msg.pose.pose.orientation.y
		qz = msg.pose.pose.orientation.z
		qw = msg.pose.pose.orientation.w
		sqx = qx**2
		sqy = qy**2
		sqz = qz**2
		sqw = qw**2
		self.robot_yaw = atan2(2*(qx*qy + qw*qz), sqw + sqx - sqy - sqz)
		if self.state == self.STATE_LOG:
			self.pose.append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

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
		self.pose = []
		self.imu =[]
		self.laser = []
		self.robot_yaw_survey = self.robot_yaw # save orientation now, it probably won't be better during the standstill
		self.state = self.STATE_LOG
		time_now = rospy.get_time()
		self.begin_log_time = time_now 
		if self.debug:
			print '%.3f Survey begin' % time_now

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
		#if self.debug:
		#	print 'GNSS Easting: %.3f Northing: %.3f Altitude: %.3f' % (easting, northing, alt)
		return (easting, northing, alt)

	def average_imu_accelerometers(self):
		n = len(self.imu)
		ax = 0.0
		ay = 0.0
		az = 0.0
		for i in xrange(n):
			ax += self.imu[i][3]
			ay += self.imu[i][4]
			az += self.imu[i][5]
		ax /= n
		ay /= n
		az /= n

		g = sqrt(ax*ax + ay*ay + az*az)
		pitch = asin(ay/-g)
		roll = atan2(ax, az)

		return (pitch, roll)

	def average_imu(self):
		n = len(self.imu)
		pitch = 0.0
		roll = 0.0
		yaw = 0.0
		for i in xrange(n):
			pitch += self.imu[i][0]
			roll += self.imu[i][1]
			yaw += self.imu[i][2]
		pitch /= n
		roll /= n
		yaw /= n
		return (pitch, roll, yaw)


	def end_log (self):
		self.state = self.STATE_IDLE
		time_now = rospy.get_time()
		error = False
		log_seconds = time_now	- self.begin_log_time
		if len(self.gnss) > 0:
			(gnss_e, gnss_n, gnss_alt) = self.average_gnss_pose()
			dist_from_wpt =  self.calc_2d_dist ([gnss_e, gnss_n], self.b)
		else:
			error = True
			dist_from_wpt = self.calc_2d_dist ([self.pose[-1][0], self.pose[-1][1]], self.b) # to avoid crash when printing

		if len(self.imu) > 0:
			#(imu_pitch, imu_roll, imu_yaw) = self.average_imu()
			(imu_pitch, imu_roll) = self.average_imu_accelerometers()
		else:
			error = True

		if error == False:
			s = '%s\t%.4f\t%.4f\t%.3f\t%.3f\t%.4f\t%.4f\t%.4f\t%.3f\t%.2f\t%.2f\t%.2f\n' %(self.b_id, self.b[0], self.b[1], time_now, log_seconds, gnss_e, gnss_n, gnss_alt, dist_from_wpt, self.robot_yaw_survey*self.rad_to_deg, imu_pitch*self.rad_to_deg, imu_roll*self.rad_to_deg)
		else:
			s= '#errror\t%.3f\t%.3f\n' % (time_now, log_seconds)

		self.save_log(s)
		self.state = self.STATE_IDLE
		if self.debug:
			print '%.3f Survey end. Dist from wpt %.3fm Yaw %.2f Pitch %.2f Roll %.2f' %  (time_now, dist_from_wpt, self.robot_yaw_survey*self.rad_to_deg, imu_pitch*self.rad_to_deg, imu_roll*self.rad_to_deg)

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


