#!/usr/bin/env python
#/****************************************************************************
# Frobit behaviour_remote_control
# Copyright (c) 2015, Kjeld Jensen <kjeld@frobomind.org>
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
2015-10-02 KJ First version
"""

# standard Python imports
from math import pi

# ROS imports
import rospy
from geometry_msgs.msg import TwistStamped

class behaviour_remote_control():
	def __init__(self):
		# get topic names
		cmd_vel_topic = rospy.get_param("~cmd_vel_pub", "/fmCommand/cmd_vel")

		# read launch file parameters
		self.vel_lin_max = rospy.get_param("~max_linear_velocity", 0.5) # [m/s]
		self.vel_ang_max = rospy.get_param("~max_angular_velocity", 0.3) # [rad/s]

		# state variables read by the mission
		self.deadman_state = False

		# local state variables
		self.joy_neutral = False
		self.joy_neutral_ok = False
		self.joy_vel_lin = 0.0
		self.joy_vel_ang = 0.0
		self.joy_vel_lin_prev = 0.0
		self.joy_vel_ang_prev = 0.0
		self.joy_vel_lin_show = 0.0
		self.joy_vel_ang_show = 0.0
		self.show_vel_tout = 0.0
		self.show_vel_interval = 1.0

		# setup manual velocity topic
		self.vel_lin = 0.0
		self.vel_ang = 0.0
		self.cmd_vel_msg = TwistStamped()
		self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, TwistStamped, queue_size=1)

	def on_rc_topic(self, msg):
		self.joy_vel_lin_prev = self.joy_vel_lin
		self.joy_vel_ang_prev = self.joy_vel_ang
		self.joy_vel_lin = self.vel_lin_max * msg.analog_joy_a_up_down/100.0
		self.joy_vel_ang = self.vel_ang_max * -msg.analog_joy_a_left_right/100.0

		# if remote control and operator is all ok
		if msg.remote_connected == True and msg.emergency_stop == False and msg.deadman_state == True:

			# check if joystick is in neutral state
			if abs(self.joy_vel_lin) < 0.0001 and abs(self.joy_vel_ang) < 0.0001:
				self.joy_neutral = True
				self.joy_neutral_ok = True
			else:
				self.joy_neutral = False

			# if joystick has been in neutral since activation
			if self.joy_neutral_ok == True:
				self.vel_lin = self.joy_vel_lin
				self.vel_ang= self.joy_vel_ang
				self.deadman_state = True 
			else:
				self.vel_lin = 0.0
				self.vel_ang = 0.0
				self.deadman_state = False

			# print status message
			if self.show_vel_tout < rospy.get_time():
				self.show_vel_tout = rospy.get_time() + self.show_vel_interval
				if abs(self.joy_vel_lin_show - self.joy_vel_lin) > 0.0001 or abs(self.joy_vel_ang_show - self.joy_vel_ang) > 0.0001:
					if self.deadman_state == True: 
						rospy.loginfo(rospy.get_name() + ": Velocity: %.2f m/s %.2f rad/s" % (self.vel_lin, self.vel_ang))
					else:
						rospy.logwarn(rospy.get_name() + ": Put joystick in neutral to activate. Velocity (disabled): %.2f m/s %.2f rad/s" % (self.joy_vel_lin, self.joy_vel_ang))
				self.joy_vel_lin_show = self.joy_vel_lin
				self.joy_vel_ang_show = self.joy_vel_ang

		else:
			self.vel_lin = 0.0
			self.vel_ang = 0.0
			self.deadman_state = False
			self.joy_neutral_ok = False 
			self.joy_vel_lin_show = 0.0
			self.joy_vel_ang_show = 0.0

	def publish_cmd_vel_message(self):
		self.cmd_vel_msg.twist.linear.x = self.vel_lin
		self.cmd_vel_msg.twist.angular.z = self.vel_ang
		self.cmd_vel_msg.header.stamp = rospy.get_rostime()
		self.cmd_vel_pub.publish(self.cmd_vel_msg)			

	def activate(self):
		self.joy_neutral_ok = False
		self.joy_vel_lin = 0.0
		self.joy_vel_ang = 0.0
		self.joy_vel_lin_show = 0.0
		self.joy_vel_ang_show = 0.0
		self.vel_lin = 0.0
		self.vel_ang = 0.0
	
	def update(self):
		self.publish_cmd_vel_message()

	def suspend(self):
		pass

