#!/usr/bin/env python
#/****************************************************************************
# Frobit lidar obstacle node 
# Copyright (c) 2015-2016, Kjeld Jensen <kjeld@frobomind.org>
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
LaserScan tutorial:
http://wiki.ros.org/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData

Revision
2015-09-17 KJ First version
"""

import rospy
from sensor_msgs.msg import LaserScan
from msgs.msg import IntArrayStamped
from math import pi, sqrt, cos, atan2

node_name = 'obstacle'

class obstacle_detect_algorithm():
	def __init__(self, obstacle_scan_width, dist_threshold_warn, dist_threshold_alarm):
		self.obstacle_scan_width = obstacle_scan_width
		self.dist_threshold_warn = dist_threshold_warn
		self.dist_threshold_alarm = dist_threshold_alarm

		self.params_set = False

	def set_params(self, range_min, range_max, angle_min, angle_max, angle_step, num_ranges):
		# save recieved parameters
		self.range_min = range_min
		self.range_max = range_max
		self.angle_min = angle_min
		self.angle_max = angle_max
		self.angle_step = angle_step

		# calculate additional parameters
		self.angle_total = self.angle_max-self.angle_min
		self.v_len = num_ranges
		self.v_c = int(self.v_len/2)

		angle_warn = atan2 (self.obstacle_scan_width, self.dist_threshold_warn)
		angle_alarm = atan2 (self.obstacle_scan_width, self.dist_threshold_alarm)

		steps_warn = int(angle_warn/self.angle_step)
		self.warn_l = self.v_c + steps_warn
		self.warn_r = self.v_c - steps_warn
		steps_alarm = int(angle_alarm/self.angle_step)
		self.alarm_l = self.v_c + steps_alarm
		self.alarm_r = self.v_c - steps_alarm

		self.alarm_thresh_right = []
		for i in xrange (self.alarm_r, self.v_c):
			self.alarm_thresh_right.append(self.dist_threshold_alarm / cos(abs(i-self.v_c)*self.angle_step))	
		self.alarm_thresh_left = []
		for i in xrange (self.v_c, self.alarm_l):
			self.alarm_thresh_left.append(self.dist_threshold_alarm / cos(abs(i-self.v_c)*self.angle_step))	

		self.warn_thresh_right = []
		for i in xrange (self.warn_r, self.v_c):
			self.warn_thresh_right.append(self.dist_threshold_warn / cos(abs(i-self.v_c)*self.angle_step))	

		self.warn_thresh_left = []
		for i in xrange (self.v_c, self.warn_l):
			self.warn_thresh_left.append(self.dist_threshold_warn / cos(abs(i-self.v_c)*self.angle_step))	

		self.params_set = True

	def new_scan(self, scan):
		if self.params_set == True:
			status_right  = 0
			for i in xrange (self.warn_r, self.v_c):
				if scan[i] < self.warn_thresh_right[i-self.warn_r]:
					status_right = 1

			for i in xrange (self.alarm_r, self.v_c):
				if scan[i] < self.alarm_thresh_right[i-self.alarm_r]:
					status_right = 2

			status_left  = 0
			for i in xrange (self.v_c, self.warn_l):
				if scan[i] < self.warn_thresh_left[i-self.v_c]:
					status_left = 1

			for i in xrange (self.v_c, self.alarm_l):
				if scan[i] < self.alarm_thresh_left[i-self.v_c]:
					status_left = 2

			return [status_left, status_right]

	def update(self):
		pass

	
class ros_node():
	def __init__(self):
		self.update_rate = 20 # [Hz]
		self.scans_skipped_cnt = 0
		self.first_scan = True

		# read parameters
		self.obstacle_scan_width = rospy.get_param("~obstacle_scan_width", 0.2) # [m (from center)] 
		self.dist_threshold_warn = rospy.get_param("~distance_threshold_warning", 1.0) # [m]
		self.dist_threshold_alarm = rospy.get_param("~distance_threshold_alarm", 0.3) # [m]
		self.scans_skip = rospy.get_param("~scans_skip", 0) # [m]

		# initialize wall finding algorithm
		self.od = obstacle_detect_algorithm(self.obstacle_scan_width, self.dist_threshold_warn, self.dist_threshold_alarm)

		# get topic names
		scan_topic = rospy.get_param("~scan_sub", "/base_scan")
		obstacle_topic = rospy.get_param("~obstacle_pub", "/fmKnowledge/obstacle")

		# setup wall pose publish topic
		self.obstacle_msg = IntArrayStamped()
		self.obstacle_msg.data = [0,0]
		self.obstacle_pub = rospy.Publisher(obstacle_topic, IntArrayStamped, queue_size=1)

		# setup subscription topic callbacks
		rospy.Subscriber(scan_topic, LaserScan, self.on_scan_topic)

		# sall updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def on_scan_topic(self, msg):
		if self.scans_skipped_cnt == self.scans_skip:
			if self.first_scan == True:
				self.first_scan = False
				#print msg.angle_min*180/pi, msg.angle_max*180/pi, msg.angle_increment*180/pi, msg.range_min, msg.range_max, len(msg.ranges)
				self.od.set_params(msg.range_min, msg.range_max, msg.angle_min, msg.angle_max, msg.angle_increment, len(msg.ranges))
			self.scans_skipped_cnt = 0
			self.obstacle_msg.data = self.od.new_scan(msg.ranges)
			self.publish_obstacle_message()
		else:
			self.scans_skipped_cnt += 1

	def publish_obstacle_message(self):
		self.obstacle_msg.header.stamp = rospy.get_rostime()
		self.obstacle_pub.publish(self.obstacle_msg)

	def updater(self):
		while not rospy.is_shutdown():
			self.od.update()
			self.r.sleep()

# main function.    
if __name__ == '__main__':
    # initialize the node and name it.
    rospy.init_node(node_name)

    # go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ros_node()
    except rospy.ROSInterruptException: pass


