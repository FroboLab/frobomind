#!/usr/bin/env python
#/****************************************************************************
# FroboMind Pose 2d Estimator: Robot Track Map Node
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
Track map node logs pose, gnss and odometry positions and show the tracks
in an interactive map plot. draw_map_details() may be edited to draw static
map details.

Revision
2013-05-09 KJ First version
"""

# ROS imports
import rospy
import numpy as np
#from tf.transformations import euler_from_quaternion
from math import sqrt, atan2, pi
from nav_msgs.msg import Odometry
from msgs.msg import gpgga_tranmerc
from robot_track_map import track_map

class track_map_node():
	def __init__(self):
		self.prev_easting = 0.0
		self.prev_northing = 0.0
		self.yaw = 0.0
		self.q = np.empty((4, ), dtype=np.float64) 

		# Get parameters
		plot_pose_pos = rospy.get_param("~plot_pose_track",False)
		plot_gnss = rospy.get_param("~plot_gnss_track",False)
		plot_odometry = rospy.get_param("~plot_odometry_track",False)
		plot_yaw = rospy.get_param("~plot_pose_yaw",False)
		offset_e = rospy.get_param("~easting_offset",0.0) # [m]
		offset_n = rospy.get_param("~northing_offset",0.0) # [m]
		self.trkpt_threshold = rospy.get_param("~trackpoint_threshold",0.1) # [m]
		map_update_frequency = rospy.get_param("~map_update_frequency", 1.0)
		map_title = rospy.get_param("~map_title", "Track")
		map_window_size = rospy.get_param("~map_window_size",5.0) # [inches]
		rospy.loginfo(rospy.get_name() + " Coordinate offset: E%.3f N%.3f" % (offset_e, offset_n))
		rospy.loginfo(rospy.get_name() + " Trackpoint threshold: %.3f m" % (self.trkpt_threshold))
 
		# setup map plot
		self.plot = track_map(plot_pose_pos, plot_gnss, plot_odometry, plot_yaw, map_title, map_window_size, offset_e, offset_n)
		self.plot.set_trackpoint_threshold (self.trkpt_threshold)

		# Get topic names
		pose_topic = rospy.get_param("~pose_sub",'/fmKnowledge/pose')
		gnss_topic = rospy.get_param("~gnss_sub",'/fmInformation/gpgga_tranmerc')
		odom_topic = rospy.get_param("~odom_sub",'/fmKnowledge/encoder_odom')

		# Setup subscription topic callbacks
		rospy.Subscriber(pose_topic, Odometry, self.on_pose_topic)
		rospy.Subscriber(gnss_topic, gpgga_tranmerc, self.on_gnss_topic)
		rospy.Subscriber(odom_topic, Odometry, self.on_odom_topic)

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

		#self.q[0] = msg.pose.pose.orientation.x
		#self.q[1] = msg.pose.pose.orientation.y
		#self.q[2] = msg.pose.pose.orientation.z
		#self.q[3] = msg.pose.pose.orientation.w
		#rot = self.quat(0,1,0,pi/3)
		#self.quaternion = self.multiply(self.quaternion, rot)
		#rot = self.quat(1,0,0,pi/3)
		#self.quaternion = self.multiply(self.quaternion, rot)
		#rot = self.quat(0,0,1,(2*pi)/3)
		#self.quaternion = self.multiply(self.quaternion, rot)
		#(roll, pitch, self.yaw) = euler_from_quaternion(self.q)
		#print 'received', self.yaw*180.0/pi

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

	# update loop
	def updater(self):
		while not rospy.is_shutdown():
			# Update map
			self.plot.append_pose_yaw(self.yaw)
			self.plot.update()

			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('robot_track_map_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = track_map_node()
    except rospy.ROSInterruptException: pass

