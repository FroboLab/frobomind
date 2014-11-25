#!/usr/bin/env python
#/****************************************************************************
# FroboMind lighthouse interface
# Copyright (c) 2014, Kjeld Jensen <kjeld@frobomind.org>
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

Revision
2014-09-23 KJ First version

1-9 blinknumre
0 slukket
10 always on

roed skal lyse fuldt indtil forbindelse

$PFLPI,
5 Hz

husk

PFLHI ogsaa

"""

# ROS imports
import rospy
from msgs.msg import nmea, gpgga_tranmerc, waypoint_navigation_status
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

class ros_node():
	def __init__(self):
		# constants

		# parameters
		update_frequency = 5

		# state variable initialization
		self.automode = False
		self.deadman = False
		self.pose = False
		self.pose_timeout = 1.0
		self.latest_pose = 0.0
		self.gps_timeout = 1.0
		self.latest_gps = 0.0
		self.gps_fix = 0

		# Get parameters
		topic_gga = rospy.get_param("~gga_sub",'/fmInformation/gpgga_tranmerc')
		topic_pose = rospy.get_param("~pose_sub",'/fmKnowledge/pose')
		topic_deadman = rospy.get_param("~deadman_sub",'/fmCommand/deadman')
		topic_automode = rospy.get_param("~automode_sub",'/fmDecision/automode')
		topic_wptnav_status = rospy.get_param("~wptnav_status_sub",'/fmInformation/wptnav_status')
		topic_lighthouse_rx = rospy.get_param("~lighthouse_sub",'/fmData/nmea_from_lighthouse')
		topic_lighthouse_tx = rospy.get_param("~lighthouse_pub",'/fmData/nmea_to_lighthouse')

		# define NMEA $PFLSL topic
		self.nmea_pflsl = nmea()
		self.nmea_pflsl.type = 'PFLSL'
		self.nmea_pflsl.length = 3
		self.nmea_pflsl.valid = True
		self.light_red = 0
		self.light_yellow = 0
		self.light_green = 0
		self.nmea_pflsl.data.append(str(self.light_red))
		self.nmea_pflsl.data.append(str(self.light_yellow))
		self.nmea_pflsl.data.append(str(self.light_green))

		# Setup subscription topic callbacks
		rospy.Subscriber(topic_gga, gpgga_tranmerc, self.on_gga_msg)
		rospy.Subscriber(topic_pose, Odometry, self.on_pose_msg)
		rospy.Subscriber(topic_deadman, Bool, self.on_deadman_msg)
		rospy.Subscriber(topic_automode, Bool, self.on_automode_msg)
		rospy.Subscriber(topic_wptnav_status, waypoint_navigation_status, self.on_wptnav_status_msg)

		# setup topic publishers
		self.nmea_pub = rospy.Publisher(topic_lighthouse_tx, nmea)

		# Call updater function
		self.r = rospy.Rate(update_frequency) # set updater frequency
		self.updater()

	# handle incoming gpgga messages
	def on_gga_msg(self, msg):
		self.latest_gps = rospy.Time.now().to_sec() 
		self.gps_fix = msg.fix
		if self.gps_fix == 5:
			self.gps_fix = 3
		elif self.gps_fix == 0:
			self.gps_fix = 1
		
	# handle incoming pose messages
	def on_pose_msg(self, msg):
		self.latest_pose = rospy.Time.now().to_sec() 

	# handle incoming deadman messages
	def on_deadman_msg(self, msg):
		self.deadman = msg.data

	# handle incoming automode messages
	def on_automode_msg(self, msg):
		self.automode = msg.data

	# handle incoming waypoint navigation status messages
	def on_wptnav_status_msg(self, msg):
		'''
		if msg.state == 1 and msg.mode != 0:
			print "%s  dist %6.2f bearing %5.1f   t_dist %5.2f t_head_err %5.1f ab_dist %.2f   lin_v %5.2f ang_v %5.2f" % (self.time_stamp(msg.header.stamp), msg.distance_to_b, msg.bearing_to_b*self.rad_to_deg, msg.target_distance, msg.target_heading_err*self.rad_to_deg, msg.distance_to_ab_line, msg.linear_speed, msg.angular_speed)

		if msg.state != self.state_prev:
			self.state_prev = msg.state
			if msg.state == 0:
				print '%s  Idle state' % self.time_stamp(msg.header.stamp)
			elif msg.state == 1:
				print '%s  Navigation state' % self.time_stamp(msg.header.stamp)
			elif msg.state == 2:
				print '%s  Wait state' % self.time_stamp(msg.header.stamp)

		if msg.mode != self.mode_prev:
			self.mode_prev = msg.mode
			if msg.mode == 0:
				print '%s  Standby mode' % self.time_stamp(msg.header.stamp)
			elif msg.mode == 1:
				print '%s  Drive mode' % self.time_stamp(msg.header.stamp)
			elif msg.mode == 2:
				print '%s  Turn mode' % self.time_stamp(msg.header.stamp)
		'''		

	def publish_frobolight_messages(self):
		self.nmea_pflsl.header.stamp = rospy.Time.now()
		self.nmea_pflsl.data[0] = str(self.light_red)
		self.nmea_pflsl.data[1] = str(self.light_yellow)
		self.nmea_pflsl.data[2] = str(self.light_green)
		self.nmea_pub.publish (self.nmea_pflsl)

	def update_lights(self):
		# update red and yellow light
		if self.latest_pose + self.pose_timeout < rospy.Time.now().to_sec():
			if self.latest_gps + self.gps_timeout < rospy.Time.now().to_sec():
				self.light_red = 1
				self.light_yellow = 0
			else:
				if self.gps_fix != 4:
					self.light_yellow = self.gps_fix
				else:
					self.gps_fix = 10
				self.light_red = 0
			self.light_green = 0
		else:
			self.light_red = 0
			self.light_yellow = 0
			if self.automode == False:
				self.light_green = 10
			else:
				self.light_green = 1
		self.publish_frobolight_messages()
	
	# update loop
	def updater(self):
		while not rospy.is_shutdown():
			self.update_lights()
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('lighthouse_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ros_node()
    except rospy.ROSInterruptException: pass

