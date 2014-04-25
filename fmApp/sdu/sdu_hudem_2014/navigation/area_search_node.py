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
"""

# imports
import rospy
import numpy as np
from std_msgs.msg import Bool, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from msgs.msg import FloatStamped, FloatArrayStamped, waypoint_navigation_status, nmea
from math import pi, atan2
from waypoint_list import waypoint_list
from waypoint_navigation import waypoint_navigation
from area_coverage_casmo import area_coverage_casmo
from wriggle import Wriggle

class WptNavNode():
	def __init__(self):
		# defines
		self.update_rate = 20 # set update frequency [Hz]
		self.update_count = 0
		self.IMPLEMENT_INVALID = -10000000.0
		self.STATE_IDLE = 0
		self.STATE_NAVIGATE = 1
		self.STATE_WAIT = 2
		self.state = self.STATE_IDLE
		self.state_prev = self.state
		self.automode_warn = False
		self.wait_after_arrival = 0.0
		self.wait_timeout = 0.0
		self.status = 0
		self.wpt = False
		self.prev_wpt = False
		self.linear_vel = 0.0
		self.angular_vel = 0.0
		self.pos = False
		self.bearing = False

		rospy.loginfo(rospy.get_name() + ": Start")
		self.quaternion = np.empty((4, ), dtype=np.float64)
		self.wii_a = False
		self.wii_a_changed = False
		self.wii_plus = False
		self.wii_plus_changed = False
		self.wii_minus = False
		self.wii_minus_changed = False
		self.wii_up = False
		self.wii_up_changed = False
		self.wii_down = False
		self.wii_down_changed = False
		self.wii_left = False
		self.wii_left_changed = False
		self.wii_right = False
		self.wii_right_changed = False
		self.wii_home = False
		self.wii_home_changed = False

		# get parameters
		self.debug = rospy.get_param("~print_debug_information", 'true') 
 		if self.debug:
			rospy.loginfo(rospy.get_name() + ": Debug enabled")
		self.status_publish_interval = rospy.get_param("~status_publish_interval", 0) 
		self.pid_publish_interval = rospy.get_param("~pid_publish_interval", 0) 

		# get topic names
		self.automode_topic = rospy.get_param("~automode_sub",'/fmDecision/automode')
		self.pose_topic = rospy.get_param("~pose_sub",'/fmKnowledge/pose')
		self.joy_topic = rospy.get_param("~joy_sub",'/fmLib/joy')
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

		turn_start_at_heading_err = rospy.get_param("~turn_start_at_heading_err", 20.0)
		turn_stop_at_heading_err = rospy.get_param("~turn_stop_at_heading_err", 2.0)
		ramp_drive_vel_at_dist = rospy.get_param("~ramp_drive_velocity_at_distance", 1.0)
		ramp_min_drive_vel = rospy.get_param("~ramp_min_drive_velocity", 0.1)
		ramp_turn_vel_at_angle = rospy.get_param("~ramp_turn_velocity_at_angle", 25.0)
		ramp_min_turn_vel = rospy.get_param("~ramp_min_turn_velocity", 0.05)
		stop_nav_at_dist = rospy.get_param("~stop_navigating_at_distance", 0.1)

		self.wptnav = waypoint_navigation(self.update_rate, self.w_dist, drive_kp, drive_ki, drive_kd, drive_ff, drive_max_output, turn_kp, turn_ki, turn_kd, turn_ff, turn_max_output, max_linear_vel, max_angular_vel, self.wpt_def_tolerance, self.wpt_def_drive_vel, self.wpt_def_turn_vel, turn_start_at_heading_err, turn_stop_at_heading_err, ramp_drive_vel_at_dist, ramp_min_drive_vel, ramp_turn_vel_at_angle, ramp_min_turn_vel, stop_nav_at_dist, self.debug)

		# configure casmo
		casmo_width = rospy.get_param("~casmo_width", 0.8)
		casmo_default_length = rospy.get_param("~casmo_default_length", 100)
		self.casmo = area_coverage_casmo()
		self.casmo.param_set_width(casmo_width)
		self.casmo.param_set_default_length(casmo_default_length)

		# Configure wriggle
		turn_angle_left = rospy.get_param("~wads_turn_angle",pi/6)
		turn_angle_right = -turn_angle_left
		speed_gain = rospy.get_param("~wads_turn_angular_speed_gain", 5)
		sensor_penalty_distance = rospy.get_param("~wads_penalty_distance", 5)
		self.wriggle = Wriggle(turn_angle_left, turn_angle_right, self.wpt_def_turn_vel, speed_gain, sensor_penalty_distance)

		# Configure wads sensor
		self.wads_topic = rospy.get_param("~wads_sub",'/fmData/nmea_from_wads')
		self.wads_threshold = rospy.get_param("~wads_threshold", 300)
		rospy.Subscriber(self.wads_topic, nmea, self.on_wads_sensor_msg)

		self.wads_value = [0.0]*10 # buffer length 10
		self.wads_value_updated = False
		self.wads_error = True

		#self.wptlist = waypoint_list()
		#self.wptlist_loaded = False

		# setup subscription topic callbacks
		rospy.Subscriber(self.automode_topic, Bool, self.on_automode_message)
		rospy.Subscriber(self.pose_topic, Odometry, self.on_pose_message)
		rospy.Subscriber(self.joy_topic, Joy, self.on_joy_message)

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
		(pos) = self.casmo.goto_next()
		self.goto_pos(pos)

	def goto_pos (self, pos):
		self.prev_wpt = self.wpt
		self.wpt = (pos[0], pos[1], 0, '', 'MCTE', self.wpt_def_tolerance, self.wpt_def_drive_vel, self.wpt_def_turn_vel)
		if self.wpt != False:
			rospy.loginfo(rospy.get_name() + ": Navigating to waypoint: %s" % self.wpt[2])
			self.wptnav.navigate(self.wpt, self.prev_wpt)
		else:
			rospy.loginfo(rospy.get_name() + ": End of waypoint list reached")
			self.wptnav.stop()

	def on_automode_message(self, msg):
		if msg.data == True: # if autonomous mode requested
			if self.state == self.STATE_IDLE:
				if self.wptnav.pose != False: # if we have a valid pose				
					self.state = self.STATE_NAVIGATE
					rospy.loginfo(rospy.get_name() + ": Switching to waypoint navigation")
					(b) = self.casmo.start(self.pos, self.bearing)
					if b != False:
						# Casmo init, set starting waypoint
						rospy.loginfo(rospy.get_name() + ": Setting casmo starting point")
						self.goto_pos(b)
					elif self.wptnav.state == self.wptnav.STATE_STANDBY:
						self.wptnav.resume()
						rospy.loginfo(rospy.get_name() + ": Resuming waypoint navigation")

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
		self.wriggle.pose_update(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
		self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
		self.bearing = yaw

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
		if int(msg.buttons[6]) != self.wii_left:
			self.wii_left = int(msg.buttons[6])
			self.wii_left_changed = True
		if int(msg.buttons[7]) != self.wii_right:
			self.wii_right =  int(msg.buttons[7])
			self.wii_right_changed = True

	def on_wads_sensor_msg(self, msg):
		if msg.type == "EBUPX" and msg.valid == True:
			self.wads_value.append (int(msg.data[0]))
			del self.wads_value[0]
			self.wads_value_updated = True

	def publish_cmd_vel_message(self):
		self.twist.header.stamp = rospy.Time.now()
		self.twist.twist.linear.x = self.linear_vel
		self.twist.twist.angular.z = self.angular_vel		
		self.cmd_vel_pub.publish (self.twist)

	def publish_implement_message(self):
		self.implement.header.stamp = rospy.Time.now()
		self.implement.data = self.wriggle.is_done()
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
			self.wptnav_status.b_id = self.wptnav.b[self.wptnav.W_ID]
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
			self.update_count += 1
			if self.wii_a == True and self.wii_a_changed == True:
				self.wii_a_changed = False
				rospy.loginfo(rospy.get_name() + ': Current position: %.3f %.3f' % (self.wptnav.pose[0], self.wptnav.pose[1]))


				self.wads_value.append (self.wads_threshold + 1.0)
				del self.wads_value[0]

			if self.wii_left == True and self.wii_left_changed == True:
				self.wii_left_changed = False
				rospy.loginfo(rospy.get_name() + ": Casmo turn left")
				(b) = self.casmo.turn_left(self.pos)
				if b:
					self.goto_pos(b)
			if self.wii_right == True and self.wii_right_changed == True:
				self.wii_right_changed = False
				(b) = self.casmo.turn_right(self.pos)
				rospy.loginfo(rospy.get_name() + ": Casmo turn right")
				if b:
					self.goto_pos(b)

			if self.state == self.STATE_NAVIGATE:
				# Start wriggle?
				if self.wads_value[-1] >= self.wads_threshold and not self.wriggle.has_sensor_penalty():
					self.wriggle.start_wriggle()
					print "Starting wriggle: %d" % self.wads_value[-1]
				
				# If wriggle is currently active, carry on
				if not self.wriggle.is_done():
					self.linear_vel = 0.0
					(state, self.angular_vel) = self.wriggle.update()
					self.publish_cmd_vel_message()

				# Else follow the waypoint navigation
				else:
					(self.status, self.linear_vel, self.angular_vel) = self.wptnav.update(rospy.get_time())
					if self.status == self.wptnav.UPDATE_ARRIVAL:
						rospy.loginfo(rospy.get_name() + ": Arrived at waypoint")
						self.goto_next_wpt()
					else:
						self.publish_cmd_vel_message()

			if self.update_count % self.update_rate == 0: # each second
				werr = True
				for i in xrange(len(self.wads_value)):
					if self.wads_value[i] != self.wads_value[0]:
						werr = False
				if werr != self.wads_error:
					self.wads_error = werr
					if self.wads_error == True:
						rospy.logerr(rospy.get_name() + ": WADS sensor error")
					else:
						rospy.logwarn(rospy.get_name() + ": Receiving data from WADS sensor")

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

