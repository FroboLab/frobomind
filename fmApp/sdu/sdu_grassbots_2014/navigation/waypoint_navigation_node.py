#!/usr/bin/env python
#/****************************************************************************
# Waypoint Navigation
# Copyright (c) 2013, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#	* Redistributions of source code must retain the above copyright
#	  notice, this list of conditions and the following disclaimer.
#	* Redistributions in binary form must reproduce the above copyright
#	  notice, this list of conditions and the following disclaimer in the
#	  documentation and/or other materials provided with the distribution.
#	* Neither the name FroboMind nor the
#	  names of its contributors may be used to endorse or promote products
#	  derived from this software without specific prior written permission.
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
2013-10-01 KJ Fixed a bug causing the velocity to always be the maximum velocity.
              Added launch file paremeters for the waypoint navigation.
2013-11-13 KJ Added new launch file parameters for waypoint defaults
              Added support for implement command and wait after wpt arriva.
2013-12-03 KJ Added ramp up which works like the previous ramp down
2014-09-10 KJ Changed from static CSV file load to dynamic ROS topic management
"""

# imports
import rospy
import numpy as np
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from msgs.msg import StringArrayStamped, FloatStamped, FloatArrayStamped, RoutePt, waypoint_navigation_status
from math import pi, atan2
from waypoint_list import waypoint_list
from waypoint_navigation import waypoint_navigation

class WptNavNode():
	def __init__(self):
		# defines
		self.update_rate = 20 # set update frequency [Hz]
		self.IMPLEMENT_INVALID = -10000000.0
		self.STATE_IDLE = 0
		self.STATE_NAVIGATE = 1
		self.STATE_WAIT = 2
		self.state = self.STATE_IDLE
		self.state_prev = self.state
		self.automode_warn = False
		self.no_route_plan_warn = False
		self.wait_after_arrival = 0.0
		self.wait_timeout = 0.0
		self.status = 0
		self.wpt = False
		self.prev_wpt = False
		self.linear_vel = 0.0
		self.angular_vel = 0.0
		self.pos = False
		self.bearing = False

		# HMI defines
		self.HMI_ID_DEADMAN = 0
		self.HMI_ID_MODE = 1
		self.HMI_ID_GOTO_WAYPOINT = 2
		self.HMI_MODE_MANUAL = 0
		self.HMI_MODE_AUTO = 1

		# route point defines
		self.ROUTEPT_CMD_DELETE = 0
		self.ROUTEPT_CMD_DELETE_THEN_APPEND = 1
		self.ROUTEPT_CMD_APPEND = 2
		self.ROUTEPT_MODE_PP = 0
		self.ROUTEPT_MODE_MCTE = 1
		self.ROUTEPT_INVALID_DATA = -1000000

		rospy.loginfo(rospy.get_name() + ": Start")
		self.quaternion = np.empty((4, ), dtype=np.float64)

		# get parameters
		self.debug = rospy.get_param("~print_debug_information", 'true') 
 		if self.debug:
			rospy.loginfo(rospy.get_name() + ": Debug enabled")
		self.status_publish_interval = rospy.get_param("~status_publish_interval", 0) 
		self.pid_publish_interval = rospy.get_param("~pid_publish_interval", 0) 

		# get topic names
		self.pose_topic = rospy.get_param("~pose_sub",'/fmKnowledge/pose')
		self.automode_topic = rospy.get_param("~automode_sub",'/fmDecision/automode')
		self.hmi_sub_topic = rospy.get_param("~hmi_sub",'/fmDecision/hmi')
		self.routept_topic = rospy.get_param("~routept_sub",'/fmPlan/route_point')
		self.cmdvel_topic = rospy.get_param("~cmd_vel_pub",'/fmCommand/cmd_vel')
		self.implement_topic = rospy.get_param("~implement_pub",'/fmCommand/implement')
		self.wptnav_status_topic = rospy.get_param("~status_pub",'/fmInformation/wptnav_status')
		self.pid_topic = rospy.get_param("~pid_pub",'/fmInformation/wptnav_pid')

		# setup publish topics
		self.cmd_vel_pub = rospy.Publisher(self.cmdvel_topic, TwistStamped)
		self.twist = TwistStamped()
		self.implement_pub = rospy.Publisher(self.implement_topic, FloatStamped)
		self.implement = FloatStamped()
		self.wptnav_status_pub = rospy.Publisher(self.wptnav_status_topic, waypoint_navigation_status)
		self.wptnav_status = waypoint_navigation_status()
		self.status_publish_count = 0
		self.pid_pub = rospy.Publisher(self.pid_topic, FloatArrayStamped)
		self.pid = FloatArrayStamped()
		self.pid_publish_count = 0

		# configure waypoint navigation
		self.w_dist = rospy.get_param("/diff_steer_wheel_distance", 0.2) # [m]
		drive_kp = rospy.get_param("~drive_kp", 1.0)
		drive_ki = rospy.get_param("~drive_ki", 0.0)
		drive_kd = rospy.get_param("~drive_kd", 0.0)
		drive_ff = rospy.get_param("~drive_feed_forward", 0.0)
		drive_max_output = rospy.get_param("~drive_max_output", 0.3)
		turn_kp = rospy.get_param("~turn_kp", 1.0)
		turn_ki = rospy.get_param("~turn_ki", 0.0)
		turn_kd = rospy.get_param("~turn_kd", 0.2)
		turn_ff = rospy.get_param("~turn_feed_forward", 0.0)
		turn_max_output = rospy.get_param("~turn_max_output", 0.5)

		max_linear_vel = rospy.get_param("~max_linear_velocity", 0.4)
		max_angular_vel = rospy.get_param("~max_angular_velocity", 0.4)

		self.wpt_def_tolerance = rospy.get_param("~wpt_default_tolerance", 0.5)
		self.wpt_def_drive_vel = rospy.get_param("~wpt_default_drive_velocity", 0.5)
		self.wpt_def_turn_vel = rospy.get_param("~wpt_default_turn_velocity", 0.3)
		self.wpt_def_wait_after_arrival = rospy.get_param("~wpt_default_wait_after_arrival", 0.0)
		self.wpt_def_implement = rospy.get_param("~wpt_default_implement_command", 0.0)

		target_ahead = rospy.get_param("~target_ahead", 1.0)
		turn_start_at_heading_err = rospy.get_param("~turn_start_at_heading_err", 20.0)
		turn_stop_at_heading_err = rospy.get_param("~turn_stop_at_heading_err", 2.0)
		ramp_drive_vel_at_dist = rospy.get_param("~ramp_drive_velocity_at_distance", 1.0)
		ramp_min_drive_vel = rospy.get_param("~ramp_min_drive_velocity", 0.1)
		ramp_turn_vel_at_angle = rospy.get_param("~ramp_turn_velocity_at_angle", 25.0)
		ramp_min_turn_vel = rospy.get_param("~ramp_min_turn_velocity", 0.05)
		stop_nav_at_dist = rospy.get_param("~stop_navigating_at_distance", 0.1)

		self.wptnav = waypoint_navigation(self.update_rate, self.w_dist, drive_kp, drive_ki, drive_kd, drive_ff, drive_max_output, turn_kp, turn_ki, turn_kd, turn_ff, turn_max_output, max_linear_vel, max_angular_vel, self.wpt_def_tolerance, self.wpt_def_drive_vel, self.wpt_def_turn_vel, target_ahead, turn_start_at_heading_err, turn_stop_at_heading_err, ramp_drive_vel_at_dist, ramp_min_drive_vel, ramp_turn_vel_at_angle, ramp_min_turn_vel, stop_nav_at_dist, self.debug)

		self.wptlist = waypoint_list()
		self. load_wpt_list() # load locally saved waypoint list if it exist

		# setup subscription topic callbacks
		rospy.Subscriber(self.automode_topic, Bool, self.on_automode_message)
		rospy.Subscriber(self.pose_topic, Odometry, self.on_pose_message)
		rospy.Subscriber(self.hmi_sub_topic, StringArrayStamped, self.on_hmi_message)
		rospy.Subscriber(self.routept_topic, RoutePt, self.on_routept_message)

		# call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def load_wpt_list (self):
		self.wptlist.load_from_csv_ne_format ('waypoints.txt')
		(numwpt, nextwpt) = self.wptlist.status()
		self.prev_wpt = False 
		self.wpt = False 
		if numwpt > 0:
			rospy.loginfo(rospy.get_name() + ": %d waypoints loaded" % numwpt)

	def update_implement_value (self):
		if self.wpt[self.wptnav.W_TASK] != self.ROUTEPT_INVALID_DATA:
			self.implement.data = self.wpt[self.wptnav.W_TASK]
		else:
			self.implement.data = self.wpt_def_implement

	def goto_first_wpt (self):
		self.prev_wpt = False
		self.wpt = self.wptlist.get_first()
		if self.wpt != False:
			self.update_implement_value()
			self.wptnav.navigate(self.wpt, self.prev_wpt)
			rospy.loginfo(rospy.get_name() + ": Navigating to waypoint: %s (distance %.2fm, bearing %.0f)" % (self.wpt[self.wptnav.W_ID], self.wptnav.dist, self.wptnav.bearing*180.0/pi))
		else:
			rospy.loginfo(rospy.get_name() + ": End of waypoint list reached")
			self.wptnav.stop()

	def goto_next_wpt (self):
		self.prev_wpt = self.wpt
		self.wpt = self.wptlist.get_next()
		if self.wpt != False:
			self.update_implement_value()
			self.wptnav.navigate(self.wpt, self.prev_wpt)
			rospy.loginfo(rospy.get_name() + ": Navigating to waypoint: %s (distance %.2fm, bearing %.0f)" % (self.wpt[self.wptnav.W_ID], self.wptnav.dist, self.wptnav.bearing*180.0/pi))
		else:
			rospy.loginfo(rospy.get_name() + ": End of waypoint list reached")
			self.wptnav.stop()

	def goto_previous_wpt (self):
		(wpt, prev_wpt) = self.wptlist.get_previous()
		if wpt != False:
			self.wpt = wpt
			self.prev_wpt = prev_wpt
			self.update_implement_value()
			self.wptnav.navigate(self.wpt, self.prev_wpt)
			rospy.loginfo(rospy.get_name() + ": Navigating to waypoint: %s (distance %.2fm, bearing %.0f)" % (self.wpt[self.wptnav.W_ID], self.wptnav.dist, self.wptnav.bearing*180.0/pi))
		else:
			rospy.loginfo(rospy.get_name() + ": This is the first waypoint")

	def on_hmi_message(self, msg):
		hmi_id = int (msg.data[0])
		if hmi_id == self.HMI_ID_DEADMAN:
			pass
		if hmi_id == self.HMI_ID_MODE:
			pass
		elif hmi_id == self.HMI_ID_GOTO_WAYPOINT:
			if self.state != self.STATE_IDLE:
				if msg.data[1] == '-':
					rospy.loginfo(rospy.get_name() + ": User selected previous waypoint")
					self.goto_previous_wpt()
				elif msg.data[1] == '+':
					rospy.loginfo(rospy.get_name() + ": User skipped waypoint")
					self.goto_next_wpt()
				
		'''
			if self.wii_a == True and self.wii_a_changed == True:
				self.wii_a_changed = False
				rospy.loginfo(rospy.get_name() + ': Current position: %.3f %.3f' % (self.wptnav.pose[0], self.wptnav.pose[1]))
		'''

	def on_routept_message(self, msg):
		if msg.cmd==self.ROUTEPT_CMD_DELETE or msg.cmd==self.ROUTEPT_CMD_DELETE_THEN_APPEND:
			self.no_route_plan_warn = False
			self.wptlist.delete_list()
		if msg.cmd==self.ROUTEPT_CMD_APPEND or msg.cmd==self.ROUTEPT_CMD_DELETE_THEN_APPEND:
			self.wptlist.append(msg.easting, msg.northing, msg.heading, msg.id, msg.nav_mode, msg.linear_vel, msg.angular_vel, msg.pause, msg.task)
			if self.state == self.STATE_NAVIGATE and self.wptnav.state == self.wptnav.STATE_STOP:
				self.goto_next_wpt()

	def on_automode_message(self, msg):
		if msg.data == True: # if autonomous mode requested
			if self.state == self.STATE_IDLE:
				if self.wptnav.pose != False: # if we have a valid pose				
					(num_wpt, next_wpt) = self.wptlist.status()
					if num_wpt > 0:
						self.state = self.STATE_NAVIGATE	
						if self.wptnav.state == self.wptnav.STATE_STANDBY:
							self.wptnav.resume()
							rospy.loginfo(rospy.get_name() + ": Resuming waypoint navigation")
						elif self.wptnav.state == self.wptnav.STATE_STOP:
							self.goto_first_wpt()
							rospy.loginfo(rospy.get_name() + ": Switching to waypoint navigation")
					else:
						if self.no_route_plan_warn == False:
							rospy.logwarn(rospy.get_name() + ": No route plan available")
							self.no_route_plan_warn = True

				else: # no valid pose yet
					if self.automode_warn == False:
						self.automode_warn = True
						rospy.logwarn(rospy.get_name() + ": Absolute pose is required for autonomous navigation")
		else: # if manual mode requested
			if self.state != self.STATE_IDLE:
				self.state = self.STATE_IDLE					
				self.wptnav.standby() 
				rospy.loginfo(rospy.get_name() + ": Switching to manual control")			
			
	def on_pose_message(self, msg):
		qx = msg.pose.pose.orientation.x
		qy = msg.pose.pose.orientation.y
		qz = msg.pose.pose.orientation.z
		qw = msg.pose.pose.orientation.w
		yaw = atan2(2*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
		self.wptnav.state_update (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, msg.twist.twist.linear.x)
	
	def publish_cmd_vel_message(self):
		self.twist.header.stamp = rospy.Time.now()
		self.twist.twist.linear.x = self.linear_vel
		self.twist.twist.angular.z = self.angular_vel		
		self.cmd_vel_pub.publish (self.twist)

	def publish_implement_message(self):
		self.implement.header.stamp = rospy.Time.now()
		self.implement_pub.publish (self.implement)

	def publish_status_message(self):
		self.wptnav_status.header.stamp = rospy.Time.now()
		self.wptnav_status.state = self.state
		if self.wptnav.pose != False:
			self.wptnav_status.easting = self.wptnav.pose[0]
			self.wptnav_status.northing = self.wptnav.pose[1]

		if self.state == self.STATE_NAVIGATE and self.wptnav.b != False:
			if  self.wptnav.state == self.wptnav.STATE_STOP or self.wptnav.state == self.wptnav.STATE_STANDBY:
				self.wptnav_status.mode = 0
			elif self.wptnav.state == self.wptnav.STATE_DRIVE:
				self.wptnav_status.mode = 1
			elif self.wptnav.state == self.wptnav.STATE_TURN:
				self.wptnav_status.mode = 2
			self.wptnav_status.b_easting = self.wptnav.b[self.wptnav.W_E]
			self.wptnav_status.b_northing = self.wptnav.b[self.wptnav.W_N]
			self.wptnav_status.a_easting = self.wptnav.a[self.wptnav.W_E]
			self.wptnav_status.a_northing = self.wptnav.a[self.wptnav.W_N]
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
			self.wptnav_status.linear_speed = self.wptnav.linear_vel
			self.wptnav_status.angular_speed = self.wptnav.angular_vel
		else:
			self.wptnav_status.mode = -1			
		self.wptnav_status_pub.publish (self.wptnav_status)

	def publish_pid_message(self):
		if self.state == self.STATE_NAVIGATE:
			self.pid.header.stamp = rospy.Time.now()
			self.pid.data = self.wptnav.pid_status
			self.pid_pub.publish (self.pid)

	def updater(self):
		while not rospy.is_shutdown():
			if self.state == self.STATE_NAVIGATE:
				(self.status, self.linear_vel, self.angular_vel) = self.wptnav.update(rospy.get_time())
				if self.status == self.wptnav.UPDATE_ARRIVAL:
					rospy.loginfo(rospy.get_name() + ": Arrived at waypoint: %s (error %.2fm)" % (self.wpt[self.wptnav.W_ID], self.wptnav.dist))

					# activate wait mode
					if self.wpt[self.wptnav.W_PAUSE] >= 0.0:
						self.wait_after_arrival = self.wpt[self.wptnav.W_PAUSE]
					else:
						self.wait_after_arrival = self.wpt_def_wait_after_arrival

					if self.wait_after_arrival > 0.01:
						self.linear_vel = 0.0
						self.angular_vel = 0.0
						self.publish_cmd_vel_message()
						self.publish_implement_message()
						rospy.loginfo(rospy.get_name() + ": Waiting %.1f seconds" % (self.wait_after_arrival))
						self.state = self.STATE_WAIT
						self.wait_timeout = rospy.get_time() + self.wait_after_arrival
					else:
						self.state = self.STATE_NAVIGATE 		
						self.goto_next_wpt()

				else:
					self.publish_cmd_vel_message()
					self.publish_implement_message()

			elif self.state == self.STATE_WAIT:
				if rospy.get_time() > self.wait_timeout:
					self.state = self.STATE_NAVIGATE 		
					self.goto_next_wpt()
				else:				
					self.linear_vel = 0.0
					self.angular_vel = 0.0
					self.publish_cmd_vel_message()
					self.publish_implement_message()

			# publish status
			if self.status_publish_interval != 0:
				self.status_publish_count += 1
				if self.status_publish_count % self.status_publish_interval == 0:
					self.publish_status_message()

			# publish pid
			if self.pid_publish_interval != 0:
				self.pid_publish_count += 1
				if self.pid_publish_count % self.pid_publish_interval == 0:
					self.publish_pid_message()
			self.r.sleep()

# Main function.	
if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('wptnav_node')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		node_class = WptNavNode()
	except rospy.ROSInterruptException:
		pass

