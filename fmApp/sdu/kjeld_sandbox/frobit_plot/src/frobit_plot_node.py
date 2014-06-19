#!/usr/bin/env python
#/****************************************************************************
# FroboMind - Frobit plot node
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
Track map node logs pose, gnss and odometry positions and show the tracks
in an interactive map plot. draw_map_details() may be edited to draw static
map details.

Revision
2013-05-09 KJ First version
2014-02-19 KJ Migrated to a frobit_plot component
2014-04-19 KJ Added support for reversing the avatar
2014-06-19 KJ Various bug fixes
"""

# ROS imports
import rospy
import numpy as np
from math import sqrt, atan2, pi
from nav_msgs.msg import Odometry
from msgs.msg import gpgga_tranmerc
from msgs.msg import waypoint_navigation_status
from frobit_plot import frobit_plot

class ROSnode():
	def __init__(self):
		self.prev_easting = 0.0
		self.prev_northing = 0.0
		self.yaw = 0.0
		self.q = np.empty((4, ), dtype=np.float64) 

		# Get parameters
		plot_pose_track = rospy.get_param("~plot_pose_track",True)
		plot_gnss_track = rospy.get_param("~plot_gnss_track",False)
		plot_odometry_track = rospy.get_param("~plot_odometry_track",False)
		plot_yaw = rospy.get_param("~plot_pose_yaw",False)
		map_trackpoint_threshold = rospy.get_param("~map_trackpoint_threshold",0.1) # [m]
		map_max_trackpoints = rospy.get_param("~map_max_trackpoints",1000) # [m]
		map_minimum_size = rospy.get_param("~map_minimum_size",10.0) # [m]
		map_easting_offset = rospy.get_param("~map_easting_offset",0.0) # [m]
		map_northing_offset = rospy.get_param("~map_northing_offset",0.0) # [m]
		map_update_frequency = rospy.get_param("~map_update_frequency", 1.0)
		map_title = rospy.get_param("~map_title", "Frobit track")
		map_window_size = rospy.get_param("~map_window_size",5.0) # [inches]
		avatar_extension_front = rospy.get_param("~avatar_extension_front",0.29) # [m] from geometric center
		avatar_extension_rear = rospy.get_param("~avatar_extension_rear",0.06) # [m] from geometric center
		avatar_extension_lateral = rospy.get_param("~avatar_extension_lateral",0.13) # [m] from geometric center
		avatar_reverse = rospy.get_param("~avatar_reverse", False) # front/rear reverse
 
		# setup map plot
		self.plot = frobit_plot(plot_pose_track, plot_gnss_track, plot_odometry_track, plot_yaw, map_trackpoint_threshold, map_max_trackpoints, map_minimum_size, map_easting_offset, map_northing_offset, map_update_frequency, map_title, map_window_size, avatar_extension_front, avatar_extension_rear, avatar_extension_lateral, avatar_reverse)

		# setup yaw plot
		self.plot_gnss_yaw = True
		self.plot_odo_yaw = True
		self.latest_absolute_yaw = 0.0
		self.latest_odo_yaw = 0.0

		# Get topic names
		pose_topic = rospy.get_param("~pose_sub",'/fmKnowledge/pose')
		gnss_topic = rospy.get_param("~gnss_sub",'/fmInformation/gpgga_tranmerc')
		odom_topic = rospy.get_param("~odom_sub",'/fmKnowledge/odometry')
		wptnav_status_topic = rospy.get_param("~wptnav_status_sub",'/fmData/wptnav_status')

		# Setup subscription topic callbacks
		rospy.Subscriber(pose_topic, Odometry, self.on_pose_topic)
		rospy.Subscriber(gnss_topic, gpgga_tranmerc, self.on_gnss_topic)
		rospy.Subscriber(odom_topic, Odometry, self.on_odom_topic)
		rospy.Subscriber(wptnav_status_topic, waypoint_navigation_status, self.on_wptnav_status_topic)

		# Call updater function
		self.r = rospy.Rate(map_update_frequency) # set updater frequency
		self.updater()

	# use this to draw static map details 
	def draw_map_details(self):
		pass

	# handle incoming pose messages
	def on_pose_topic(self, msg):
		self.plot.append_pose_position(msg.pose.pose.position.x, msg.pose.pose.position.y)
		qx = msg.pose.pose.orientation.x
		qy = msg.pose.pose.orientation.y
		qz = msg.pose.pose.orientation.z
		qw = msg.pose.pose.orientation.w
		self.yaw = atan2(2*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)

	# handle incoming GNSS messages
	def on_gnss_topic(self, msg):
		# dirty hack to perform simple filtering of singular extreme outliers to maintain map visibility
		if 	self.prev_easting != 0.0 or self.prev_northing != 0.0:
			ddist = sqrt((msg.easting - self.prev_easting)**2 + (msg.northing - self.prev_northing)**2)
			if ddist < 10.0: # if distance larger than 10 meter
				self.plot.append_gnss_position(msg.easting, msg.northing)
		self.prev_easting = msg.easting
		self.prev_northing = msg.northing

	# handle incoming odometry messages
	def on_odom_topic(self, msg):
		self.plot.append_odometry_position(msg.pose.pose.position.x, msg.pose.pose.position.y)
		qx = msg.pose.pose.orientation.x
		qy = msg.pose.pose.orientation.y
		qz = msg.pose.pose.orientation.z
		qw = msg.pose.pose.orientation.w
		self.latest_odo_yaw = atan2(2*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)

	# handle incoming waypoint navigation status messages
	def on_wptnav_status_topic(self, msg):
		self.plot.set_wptnav (msg.mode, msg.a_easting, msg.a_northing, msg.b_easting, msg.b_northing, msg.target_easting, msg.target_northing)

	# update loop
	def updater(self):
		while not rospy.is_shutdown():
			# Update map
			self.plot.append_pose_yaw(self.yaw)
			if self.plot_odo_yaw:
				self.plot.append_odo_yaw (self.latest_odo_yaw)

			self.plot.update()
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('frobit_plot_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ROSnode()
    except rospy.ROSInterruptException: pass

