#!/usr/bin/env python
#/****************************************************************************
# Keyboard mission script
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
This mission file provides simple mission handling based on keyboard input
from the user:

a(uto)      Enter autonomous mode
m(anual)    Enter manual mode

e(nable)    Enable deadman signal
Space:      Disable deadman signal

In manual mode
s(top)      Stop the robot by setting the velocity to zero
Arrows      Set the velocity

In autonomous mode
Up arrow    Go to the next waypoint
Down arrow  Go to the previous waypoint

Revision
2013-11-06 KJ First version
2014-09-08 KJ Added support for /fmDecision/hmi (previous/next waypoint)
"""

import rospy
from std_msgs.msg import Bool, Char
from geometry_msgs.msg import TwistStamped
from msgs.msg import StringArrayStamped

class mission_node():
	def __init__(self):
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

		# keyboard interface
		self.KEY_ESC = 27
		self.KEY_SECOND = 91
		self.KEY_SPACE = 32
		self.KEY_a = 97
		self.KEY_e = 101
		self.KEY_m = 109
		self.KEY_s = 115
		self.KEY_ARROW_UP = 65
		self.KEY_ARROW_DOWN = 66
		self.KEY_ARROW_RIGHT = 67
		self.KEY_ARROW_LEFT = 68
		self.esc_key = False
		self.second_key = False

		# read parameters
		self.vel_lin_max = rospy.get_param("~max_linear_velocity", 0.5) # [m/s]
		self.vel_ang_max = rospy.get_param("~max_angular_velocity", 0.3) # [rad/s]
		self.vel_lin_step = rospy.get_param("~linear_velocity_step", 0.1) # [m/s]
		self.vel_ang_step = rospy.get_param("~angular_velocity_step", 0.1) # [rad/s]

		# get topic names
		kbd_topic = rospy.get_param("~keyboard_sub", "/fmHMI/keyboard")
		deadman_topic = rospy.get_param("~deadman_pub", "/fmCommand/deadman")
		hmi_pub_topic = rospy.get_param("~hmi_pub",'/fmDecision/hmi')
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
		self.hmi_msg.data = ['', ''] # initialize string array
		self.hmi_pub = rospy.Publisher(hmi_pub_topic, StringArrayStamped)
		
		# setup manual velocity topic
		self.vel_lin = 0.0
		self.vel_ang = 0.0
		self.cmd_vel_msg = TwistStamped()
		self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, TwistStamped)

		# setup subscription topic callbacks
		rospy.Subscriber(kbd_topic, Char, self.on_kbd_topic)

		# sall updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def on_kbd_topic(self, msg):
		if self.esc_key == False:
			if msg.data == self.KEY_ESC: # the following key will be a secondary key
				self.esc_key = True
			elif msg.data == self.KEY_SPACE: # disable actuation
				self.vel_lin = 0.0
				self.vel_ang = 0.0	 
				if self.deadman_state == True:
					rospy.logwarn(rospy.get_name() + ": Disabling actuation")
					self.deadman_state = False
			elif msg.data == self.KEY_e: # enable actuation
				if self.deadman_state == False:
					rospy.logwarn(rospy.get_name() + ": Enabling actuation")
					self.deadman_state = True
			elif msg.data == self.KEY_a:
				if self.state == self.STATE_MANUAL:
					rospy.logwarn(rospy.get_name() + ": Switching to autonomous mode")
					self.state = self.STATE_AUTO
			elif msg.data == self.KEY_m:
				if self.state == self.STATE_AUTO:
					rospy.logwarn(rospy.get_name() + ": Switching to manual mode")
					self.state = self.STATE_MANUAL
					self.vel_lin = 0.0
					self.vel_ang = 0.0
			elif msg.data == self.KEY_s:
				if self.state == self.STATE_MANUAL:
					self.vel_lin = 0.0
					self.vel_ang = 0.0
		else: # this key is a secondary key
			if msg.data == self.KEY_SECOND: # the following key will be a secondary key
				self.second_key = True
			elif self.second_key == True:
				self.esc_key = False
				self.second_key = False
				if self.state == self.STATE_MANUAL:
					if msg.data == self.KEY_ARROW_UP:
						self.vel_lin += self.vel_lin_step
						if self.vel_lin > self.vel_lin_max:
							self.vel_lin = self.vel_lin_max
					elif msg.data == self.KEY_ARROW_DOWN:
						self.vel_lin -= self.vel_lin_step
						if self.vel_lin < -self.vel_lin_max:
							self.vel_lin = -self.vel_lin_max
					elif msg.data == self.KEY_ARROW_LEFT:
						self.vel_ang += self.vel_ang_step
						if self.vel_ang > self.vel_ang_max:
							self.vel_ang = self.vel_ang_max
					elif msg.data == self.KEY_ARROW_RIGHT:
						self.vel_ang -= self.vel_ang_step
						if self.vel_ang < -self.vel_ang_max:
							self.vel_ang = -self.vel_ang_max
					if self.deadman_state == True:
						rospy.loginfo(rospy.get_name() + ": Velocity: %.1f m/s %.1f rad/s" % (self.vel_lin, self.vel_ang))
					else:
						rospy.loginfo(rospy.get_name() + ": Velocity (disabled): %.1f m/s %.1f rad/s" % (self.vel_lin, self.vel_ang))
					if abs (self.vel_lin) < 0.0001: # make sure zero is zero!
						self.vel_lin = 0.0
					if abs (self.vel_ang) < 0.0001:
						self.vel_ang = 0.0
				elif self.state == self.STATE_AUTO:
					if msg.data == self.KEY_ARROW_UP:
						self.hmi_msg.header.stamp = rospy.Time.now()
						self.hmi_msg.data[0] = '%d' % self.HMI_ID_GOTO_WAYPOINT
						self.hmi_msg.data[1] = '+'
						self.hmi_pub.publish (self.hmi_msg)
					elif msg.data == self.KEY_ARROW_DOWN:
						self.hmi_msg.header.stamp = rospy.Time.now()
						self.hmi_msg.data[0] = '%d' % self.HMI_ID_GOTO_WAYPOINT
						self.hmi_msg.data[1] = '-'
						self.hmi_pub.publish (self.hmi_msg)

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

	def updater(self):
		while not rospy.is_shutdown():
			self.publish_deadman_message()
			self.publish_automode_message()
			if self.deadman_state == True and self.state == self.STATE_MANUAL:
				self.publish_cmd_vel_message()
			self.r.sleep()

# main function.    
if __name__ == '__main__':
    # initialize the node and name it.
    rospy.init_node('keyboard_mission')

    # go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = mission_node()
    except rospy.ROSInterruptException: pass


