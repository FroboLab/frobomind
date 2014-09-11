#!/usr/bin/env python
#/****************************************************************************
# Wiimote mission script
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
This mission file provides simple mission handling based on input from a
Nintendo Wiimote

Revision
2013-11-14 KJ First version
2014-09-08 KJ Added support for /fmDecision/hmi (previous/next waypoint)
"""

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from msgs.msg import StringArrayStamped
from math import pi

class mission_node():
	def __init__(self):
		# fixed parameters
		self.update_rate = 20 # [Hz]

		# robot state
		self.STATE_AUTO = 0
		self.STATE_MANUAL = 1
		self.state = self.STATE_MANUAL

		# HMI id's
		self.HMI_ID_DEADMAN = 0
		self.HMI_ID_MODE = 1
		self.HMI_ID_GOTO_WAYPOINT = 2
		self.HMI_MODE_MANUAL = 0
		self.HMI_MODE_AUTO = 1

		# wiimote state
		self.wii_1 = False
		self.wii_1_changed = False
		self.wii_2 = False
		self.wii_2_changed = False
		self.wii_a = False
		self.wii_a_changed = False
		self.wii_b = False
		self.wii_b_changed = False
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

		self.wii_angle_min = 3.0*pi/180.0
		self.wii_angle_max = 60.0*pi/180.0
		self.wii_angle_diff = self.wii_angle_max - self.wii_angle_min

		# read parameters
		self.vel_lin_user_max = rospy.get_param("~linear_velocity_default", 0.5) # [m/s]
		self.vel_lin_user_step = rospy.get_param("~linear_velocity_step", 0.1) # [m/s]
		self.vel_ang_user_max = rospy.get_param("~angular_velocity_default", 0.4) # [rad/s]
		self.vel_ang_user_step = rospy.get_param("~angular_velocity_step", 0.1) # [rad/s]

		self.vel_lin_max = rospy.get_param("~max_linear_velocity", 1.0) # [m/s]
		self.vel_ang_max = rospy.get_param("~max_angular_velocity", 0.5) # [rad/s]
		if self.vel_lin_user_max > self.vel_lin_max:
			self.vel_lin_user_max = self.vel_lin_max
		if self.vel_ang_user_max > self.vel_ang_max:
			self.vel_ang_user_max = self.vel_ang_max

		acc_lin_max = rospy.get_param("~max_linear_acceleration", 2.0) # [m/s^2]
		acc_ang_max = rospy.get_param("~max_angular_acceleration", pi) # [rad/s^2]
		self.acc_lin_max_step = acc_lin_max/(self.update_rate * 1.0)		
		self.acc_ang_max_step = acc_ang_max/(self.update_rate * 1.0)		
		dec_lin_max = rospy.get_param("~max_linear_deceleration", 2.0) # [m/s^2]
		dec_ang_max = rospy.get_param("~max_angular_deceleration", pi) # [rad/s^2]
		self.dec_lin_max_step = dec_lin_max/(self.update_rate * 1.0)		
		self.dec_ang_max_step = dec_ang_max/(self.update_rate * 1.0)		

		# get topic names
		hmi_pub_topic = rospy.get_param("~hmi_pub",'/fmDecision/hmi')
		joy_topic = rospy.get_param("~wiimote_sub",'/fmLib/joy')
		deadman_topic = rospy.get_param("~deadman_pub", "/fmCommand/deadman")
		automode_topic = rospy.get_param("~automode_pub", "/fmDecision/automode")
		cmd_vel_topic = rospy.get_param("~cmd_vel_pub", "/fmCommand/cmd_vel")

		# setup deadman publish topic
		self.deadman_state = False
		self.deadman_msg = Bool()
		self.deadman_pub = rospy.Publisher(deadman_topic, Bool)

		# setup automode publish topic
		self.automode_msg = Bool()
		self.automode_pub = rospy.Publisher(automode_topic, Bool)

		# setup HMI publish topic
		self.hmi_msg = StringArrayStamped()
		self.hmi_pub = rospy.Publisher(hmi_pub_topic, StringArrayStamped)
		
		# setup manual velocity topic
		self.vel_lin_user = 0.0
		self.vel_ang_user = 0.0
		self.vel_lin = 0.0
		self.vel_ang = 0.0
		self.cmd_vel_msg = TwistStamped()
		self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, TwistStamped)

		# setup subscription topic callbacks
		rospy.Subscriber(joy_topic, Joy, self.on_joy_topic)

		# call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def limit_angle (self, angle):
		if angle > 0:
			if angle < self.wii_angle_min:
				angle = 0
			elif angle > self.wii_angle_max:
				angle = self.wii_angle_max
		else:
			if angle > -self.wii_angle_min:
				angle = 0
			elif angle < -self.wii_angle_max:
				angle = -self.wii_angle_max
		return angle

	def on_joy_topic(self, msg):
		if int(msg.buttons[0]) != self.wii_1:
			self.wii_1 =  int(msg.buttons[0])
			self.wii_1_changed = True
		if int(msg.buttons[1]) != self.wii_2:
			self.wii_2 =  int(msg.buttons[1])
			self.wii_2_changed = True
		if int(msg.buttons[2]) != self.wii_a:
			self.wii_a =  int(msg.buttons[2])
			self.wii_a_changed = True
		if int(msg.buttons[3]) != self.wii_b:
			self.wii_b =  int(msg.buttons[3])
			self.wii_b_changed = True
		if int(msg.buttons[6]) != self.wii_left:
			self.wii_left =  int(msg.buttons[6])
			self.wii_left_changed = True
		if int(msg.buttons[7]) != self.wii_right:
			self.wii_right =  int(msg.buttons[7])
			self.wii_right_changed = True
		if int(msg.buttons[8]) != self.wii_up:
			self.wii_up =  int(msg.buttons[8])
			self.wii_up_changed = True
		if int(msg.buttons[9]) != self.wii_down:
			self.wii_down =  int(msg.buttons[9])
			self.wii_down_changed = True
		if int(msg.buttons[10]) != self.wii_home:
			self.wii_home =  int(msg.buttons[10])
			self.wii_home_changed = True

		roll = self.limit_angle (-float(msg.axes[0])*pi/20.0)
		pitch =  self.limit_angle (float(msg.axes[1])*pi/20.0)

		if self.wii_up_changed == True:
			self.wii_up_changed = False
			if self.state = self.STATE_MANUAL:
				self.vel_lin_user_max += self.vel_lin_user_step
				if self.vel_lin_user_max > self.vel_lin_max:
					self.vel_lin_user_max = self.vel_lin_max
			else:
				self.hmi_msg.header.stamp = rospy.Time.now()
				self.hmi_msg.data[0] = '%d' % self.HMI_ID_GOTO_WAYPOINT
				self.hmi_msg.data[1] = '+'
				self.hmi_pub.publish (self.hmi_msg)

		if self.wii_down_changed == True:
			self.wii_down_changed = False
			if self.state = self.STATE_MANUAL:
				self.vel_lin_user_max -= self.vel_lin_user_step
				if self.vel_lin_user_max < self.vel_lin_user_step:
					self.vel_lin_user_max = self.vel_lin_user_step
			else:
				self.hmi_msg.header.stamp = rospy.Time.now()
				self.hmi_msg.data[0] = '%d' % self.HMI_ID_GOTO_WAYPOINT
				self.hmi_msg.data[1] = '-'
				self.hmi_pub.publish (self.hmi_msg)

		if self.wii_left_changed == True:
			self.wii_left_changed = False
			if self.state = self.STATE_MANUAL:
				self.vel_ang_user_max -= self.vel_ang_user_step
				if self.vel_ang_user_max < self.vel_ang_user_step:
					self.vel_ang_user_max = self.vel_ang_user_step
		
		if self.wii_right_changed == True:
			if self.state = self.STATE_MANUAL:
				self.wii_right_changed = False
				self.vel_ang_user_max += self.vel_ang_user_step
				if self.vel_ang_user_max > self.vel_ang_max:
					self.vel_ang_user_max = self.vel_ang_max

		if pitch >= 0:
			self.vel_lin_user = self.vel_lin_user_max*(pitch-self.wii_angle_min)/self.wii_angle_diff
		else:
			self.vel_lin_user = self.vel_lin_user_max*(pitch+self.wii_angle_min)/self.wii_angle_diff

		if roll >= 0:
			self.vel_ang_user = self.vel_ang_user_max*(roll-self.wii_angle_min)/self.wii_angle_diff
		else:
			self.vel_ang_user = self.vel_ang_user_max*(roll+self.wii_angle_min)/self.wii_angle_diff

		self.deadman_state = self.wii_b
		if self.wii_1_changed:
			self.state = self.STATE_AUTO
			self.wii_1_changed = False
		elif self.wii_2_changed:
			self.reset_vel()
			self.state = self.STATE_MANUAL
			self.wii_2_changed = False

	def publish_deadman_message(self):
		self.deadman_msg = self.deadman_state
		self.deadman_pub.publish (self.deadman_msg)

	def publish_automode_message(self):
		self.automode_msg = (self.state == self.STATE_AUTO)
		self.automode_pub.publish (self.automode_msg)

	def publish_cmd_vel_message(self):
		self.cmd_vel_msg.header.stamp = rospy.Time.now()
		self.cmd_vel_msg.twist.linear.x = self.vel_lin
		self.cmd_vel_msg.twist.angular.z = self.vel_ang
		self.cmd_vel_pub.publish(self.cmd_vel_msg)

	def accelerate_vel (self, vel_actual, vel_desired, max_step):
		# update velocity while respecting max acceleration
		if vel_actual < vel_desired:
			vel_actual += max_step
			if vel_actual > vel_desired:
				vel_actual = vel_desired
		elif vel_actual > vel_desired:
			vel_actual -= max_step
			if vel_actual < vel_desired:
				vel_actual = vel_desired
		return vel_actual

	def reset_vel (self):
		self.vel_lin = 0.0
		self.vel_ang = 0.0
		self.vel_lin_user = 0.0
		self.vel_ang_user = 0.0

	def updater(self):
		while not rospy.is_shutdown():
			self.publish_deadman_message()
			self.publish_automode_message()
			if self.state == self.STATE_MANUAL:
				if abs(self.vel_lin) < abs(self.vel_lin_user):
					self.vel_lin = self.accelerate_vel (self.vel_lin, self.vel_lin_user, self.acc_lin_max_step)
				else:
					self.vel_lin = self.accelerate_vel (self.vel_lin, self.vel_lin_user, self.dec_lin_max_step)
				if abs(self.vel_ang) < abs(self.vel_ang_user):
					self.vel_ang = self.accelerate_vel (self.vel_ang, self.vel_ang_user, self.acc_ang_max_step)
				else:
					self.vel_ang = self.accelerate_vel (self.vel_ang, self.vel_ang_user, self.dec_ang_max_step)

				self.publish_cmd_vel_message()
			self.r.sleep()

# main function.    
if __name__ == '__main__':
    # initialize the node and name it.
    rospy.init_node('wiimote_mission')

    # go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = mission_node()
    except rospy.ROSInterruptException: pass


