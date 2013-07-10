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
from msgs.msg import waypoint_navigation_status
from math import pi, atan2
from waypoint_list import waypoint_list
from waypoint_navigation import waypoint_navigation

class WaypointNavigationNode():
	def __init__(self):
		# defines
		self.update_rate = 20 # set update frequency [Hz]
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
		self.wii_home = False
		self.wii_home_changed = False
		self.wii_up = False
		self.wii_up_changed = False
		self.wii_down = False
		self.wii_down_changed = False

		# get parameters
		self.debug = rospy.get_param("~print_debug_information", 'true') 
 		if self.debug:
			rospy.loginfo(rospy.get_name() + ": Debug enabled")
		self.status_publish_interval = rospy.get_param("~status_publish_interval", '0') 

		# get topic names
		self.automode_topic = rospy.get_param("~automode_sub",'/fmDecisionMakers/automode')
		self.pose_topic = rospy.get_param("~pose_sub",'/fmKnowledge/pose')
		self.joy_topic = rospy.get_param("~joy_sub",'/joy')
		self.cmdvel_topic = rospy.get_param("~cmd_vel_pub",'/fmCommand/cmd_vel')
		self.wptnav_status_topic = rospy.get_param("~status_pub",'/fmData/wptnav_status')

		# setup subscription topic callbacks
		rospy.Subscriber(self.automode_topic, Bool, self.on_automode_message)
		rospy.Subscriber(self.pose_topic, Odometry, self.on_pose_message)
		rospy.Subscriber(self.joy_topic, Joy, self.on_joy_message)

		# setup publish topics
		self.cmd_vel_pub = rospy.Publisher(self.cmdvel_topic, TwistStamped)
		self.twist = TwistStamped()
		self.wptnav_status_pub = rospy.Publisher(self.wptnav_status_topic, waypoint_navigation_status)
		self.wptnav_status = waypoint_navigation_status()
		self.status_publish_count = 0

		# configure waypoint navigation
		self.wptlist = waypoint_list()
		self.wptnav = waypoint_navigation(self.update_rate, self.debug)
		self.wptlist_loaded = False

		# call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def load_wpt_list (self):
		self.wptlist.load_from_csv_ne_format ('waypoints.txt')
		(numwpt, nextwpt) = self.wptlist.status()
		self.prev_wpt = False 
		self.wpt = False 
		rospy.loginfo(rospy.get_name() + ": %d waypoints loaded" % numwpt)

	def goto_next_wpt (self):
		self.prev_wpt = self.wpt
		self.wpt = self.wptlist.get_next()
		if self.wpt != False:
			rospy.loginfo(rospy.get_name() + ": Navigating to waypoint: %s" % self.wpt[2])
			self.wptnav.navigate(self.wpt, self.prev_wpt)
		else:
			rospy.loginfo(rospy.get_name() + ": End of waypoint list reached")
			self.wptnav.stop()

	def goto_previous_wpt (self):
		(wpt, prev_wpt) = self.wptlist.get_previous()
		if wpt != False:
			self.wpt = wpt
			self.prev_wpt = prev_wpt
			rospy.loginfo(rospy.get_name() + ": Navigating to waypoint: %s" % self.wpt[2])
			self.wptnav.navigate(self.wpt, self.prev_wpt)
		else:
			rospy.loginfo(rospy.get_name() + ": This is the first waypoint")

	def on_automode_message(self, msg):
		self.automode = msg.data
		if self.automode != self.automode_prev:
			self.automode_prev = self.automode
			if self.automode:
				rospy.loginfo(rospy.get_name() + ": Switching to waypoint navigation")
				if self.wptlist_loaded == False:
					rospy.loginfo(rospy.get_name() + ": Loading waypoint list")
					self.load_wpt_list()				
					self.goto_next_wpt()
					self.wptlist_loaded = True
				elif self.wptnav.state == self.wptnav.STATE_STANDBY:
					self.wptnav.resume()
					rospy.loginfo(rospy.get_name() + ": Resuming waypoint navigation")
			else:
				self.wptnav.standby() 
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
		if int(msg.buttons[8]) != self.wii_up:
			self.wii_up =  int(msg.buttons[8])
			self.wii_up_changed = True
		if int(msg.buttons[9]) != self.wii_down:
			self.wii_down =  int(msg.buttons[9])
			self.wii_down_changed = True
		if int(msg.buttons[10]) != self.wii_home:
			self.wii_home =  int(msg.buttons[10])
			self.wii_home_changed = True
	
	def publish_cmd_vel_message(self):
		self.twist.header.stamp = rospy.Time.now()
		self.twist.twist.linear.x = self.linear_speed
		self.twist.twist.angular.z = self.angular_speed		
		self.cmd_vel_pub.publish (self.twist)

	def publish_status_message(self):
		self.wptnav_status.header.stamp = rospy.Time.now()
		if self.wptnav.pose != False:
			self.wptnav_status.easting = self.wptnav.pose[0]
			self.wptnav_status.northing = self.wptnav.pose[1]
		else:
			self.wptnav_status.easting = 0.0
			self.wptnav_status.northing = 0.0
		if self.automode != False and self.wptnav.b != False:
			if  self.wptnav.state == self.wptnav.STATE_STOP or self.wptnav.state == self.wptnav.STATE_STANDBY:
				self.wptnav_status.mode = 0
			elif self.wptnav.state == self.wptnav.STATE_DRIVE_INIT or self.wptnav.state == self.wptnav.STATE_DRIVE:
				self.wptnav_status.mode = 1
			elif self.wptnav.state == self.wptnav.STATE_TURN_INIT or self.wptnav.state == self.wptnav.STATE_TURN:
				self.wptnav_status.mode = 2
			self.wptnav_status.b_easting = self.wptnav.b[0]
			self.wptnav_status.b_northing = self.wptnav.b[1]
			self.wptnav_status.a_easting = self.wptnav.a[0]
			self.wptnav_status.a_northing = self.wptnav.a[1]
			self.wptnav_status.distance_to_b = self.wptnav.dist
			self.wptnav_status.bearing_to_b = self.wptnav.bearing
			self.wptnav_status.heading_err = self.wptnav.heading_err
			self.wptnav_status.distance_to_ab_line = self.wptnav.ab_dist_to_pose
			if self.wptnav.target != False:
				self.wptnav_status.target_easting = self.wptnav.target[0]
				self.wptnav_status.target_northing = self.wptnav.target[1]
				self.wptnav_status.target_distance = self.wptnav.target_dist
				self.wptnav_status.target_bearing = self.wptnav.target_bearing
				self.wptnav_status.target_heading_err = self.wptnav.target_heading_err
			else:	
				self.wptnav_status.target_easting = 0.0
				self.wptnav_status.target_northing = 0.0
				self.wptnav_status.target_distance = 0.0
				self.wptnav_status.target_bearing = 0.0
				self.wptnav_status.target_heading_err = 0.0
			self.wptnav_status.linear_speed = self.wptnav.linear_speed
			self.wptnav_status.angular_speed = self.wptnav.angular_speed
		else:
			self.wptnav_status.mode = -1			
		self.wptnav_status_pub.publish (self.wptnav_status)

	def updater(self):
		while not rospy.is_shutdown():
			if self.wii_a == True and self.wii_a_changed == True:
				self.wii_a_changed = False
				rospy.loginfo(rospy.get_name() + ': Current position: %.3f %.3f' % (self.wptnav.pose[0], self.wptnav.pose[1]))
			if self.wii_home == True and self.wii_home_changed == True:
				self.wii_home_changed = False
				rospy.loginfo(rospy.get_name() + ": User reloaded waypoint list")
				self.load_wpt_list()				
				self.goto_next_wpt()
			if self.wii_up == True and self.wii_up_changed == True:
				self.wii_up_changed = False
				rospy.loginfo(rospy.get_name() + ": Moving to next waypoint")
				if self.wptnav.state == self.wptnav.STATE_STANDBY:
					self.wptnav.resume()
				self.goto_next_wpt()
			if self.wii_down == True and self.wii_down_changed == True:
				self.wii_down_changed = False
				rospy.loginfo(rospy.get_name() + ": User selected previous waypoint")
				self.goto_previous_wpt()

			if self.automode:
				if self.wptnav.pose != False:
					ros_time = rospy.Time.now()
					time = ros_time.secs + ros_time.nsecs*1e-9
					(self.status, self.linear_speed, self.angular_speed) = self.wptnav.update(time)
					if self.status == self.wptnav.UPDATE_ARRIVAL:
						self.wptnav.standby()
					else:
						self.publish_cmd_vel_message()
				else:
					self.linear_speed = 0.0
					self.angular_speed = 0.0
					self.publish_cmd_vel_message()
			if self.status_publish_interval != 0:
				self.status_publish_count += 1
				if self.status_publish_count % self.status_publish_interval == 0:
					self.publish_status_message()
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

