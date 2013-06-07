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
2013-06-06 KJ First version
"""

# imports
import rospy
import numpy as np
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from math import pi, atan2
from waypoint_list import waypoint_list
from waypoint_navigation import waypoint_navigation

class WaypointNavigationNode():
	def __init__(self):
		# defines
		self.updater_rate = 10 # set updater frequency [Hz]
		self.automode = False
		self.automode_prev = False
		self.status = 0
		self.wpt = False
		self.prev_wpt = False
		self.linear_speed = 0.0
		self.angular_speed = 0.0

		rospy.loginfo(rospy.get_name() + ": Start")
		self.quaternion = np.empty((4, ), dtype=np.float64)
		self.wii_a = False
		self.wii_a_changed = False

		# get parameters
		self.debug = (rospy.get_param("~print_debug_information", "true") == "true")
 		if self.debug:
			rospy.loginfo(rospy.get_name() + ": Debug enabled")

		# get topic names
		self.automode_topic = rospy.get_param("~automode_sub",'/fmDecisionMakers/automode')
		self.pose_topic = rospy.get_param("~pose_sub",'/fmKnowledge/pose')
		self.joy_topic = rospy.get_param("~joy_sub",'/joy')
		self.cmdvel_topic = rospy.get_param("~cmd_vel_pub",'/fmCommand/cmd_vel')

		# setup subscription topic callbacks
		rospy.Subscriber(self.automode_topic, Bool, self.on_automode_message)
		rospy.Subscriber(self.pose_topic, Odometry, self.on_pose_message)
		rospy.Subscriber(self.joy_topic, Joy, self.on_joy_message)

		# setup publish topics
		self.cmd_vel_pub = rospy.Publisher(self.cmdvel_topic, TwistStamped)
		self.twist = TwistStamped()

		# configure waypoint navigation
		self.wptlist = waypoint_list()
		self.wptnav = waypoint_navigation()

		# call updater function
		self.r = rospy.Rate(self.updater_rate)
		self.updater()

	def load_wpt_list (self):
		self.wptlist.load_from_csv_ne_format ('waypoints_square.txt')
		(numwpt, nextwpt) = self.wptlist.status()
		rospy.loginfo(rospy.get_name() + ": %d waypoints loaded" % numwpt)

	def goto_next_wpt (self):
		self.prev_wpt = self.wpt
		self.wpt = self.wptlist.get_next()
		if self.wpt != False:
			rospy.loginfo(rospy.get_name() + ": Navigating to waypoint: %s" % self.wpt[2])
			self.wptnav.navigate(self.wpt, self.prev_wpt)
		else:
			rospy.loginfo(rospy.get_name() + ": End of waypoint list reached")

	def on_automode_message(self, msg):
		self.automode = msg.data
		if self.automode != self.automode_prev:
			self.automode_prev = self.automode
			if self.automode:
				self.load_wpt_list()				
				self.goto_next_wpt()
				rospy.loginfo(rospy.get_name() + ": Switching to waypoint navigation")
			else:
				self.wptnav.stop() 
				rospy.loginfo(rospy.get_name() + ": Switching to Wiimote control")			

	def on_pose_message(self, msg):
		qx = msg.pose.pose.orientation.x
		qy = msg.pose.pose.orientation.y
		qz = msg.pose.pose.orientation.z
		qw = msg.pose.pose.orientation.w
		yaw = atan2(2*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
		self.wptnav.pose_update (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

	def on_joy_message(self, msg):
		if int(msg.buttons[2]) != self.wii_a:
			self.wii_a =  int(msg.buttons[2])
			self.wii_a_changed = True
	
	def publish_cmd_vel_message(self):
		self.twist.header.stamp = rospy.Time.now()
		self.twist.twist.linear.x = self.linear_speed
		self.twist.twist.angular.z = self.angular_speed		
		self.cmd_vel_pub.publish (self.twist)

	def updater(self):
		while not rospy.is_shutdown():
			if self.automode:
				(self.status, self.linear_speed, self.angular_speed) = self.wptnav.update()
				if self.status == self.wptnav.UPDATE_ARRIVAL:
					wpt = self.wptlist.get_next()
					if wpt != False:
						rospy.loginfo(rospy.get_name() + ": Navigating to waypoint: %s" % wpt[2])
						self.wptnav.navigate(wpt, False)
					else:
						rospy.loginfo(rospy.get_name() + ": End of waypoint list reached")		
						self.wptnav.stop() 
				else:
					self.publish_cmd_vel_message()
				self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('waypoint_navigation_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = WaypointNavigationNode()
    except rospy.ROSInterruptException:
		pass

