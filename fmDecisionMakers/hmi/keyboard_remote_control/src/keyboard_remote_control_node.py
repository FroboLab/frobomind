#!/usr/bin/env python
#/****************************************************************************
# Keyboard remote control node
# Copyright (c) 2013-2015, Kjeld Jensen <kjeld@frobomind.org>
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
This remote control component provides a standard remote control output based
on keyboard input from the user:

a(uto)   Enter autonomous mode
m(anual) Enter manual mode

e(nable) Enable deadman signal
Space:   Disable deadman signal and enter manual mode

arrow keys are used to step towards max and minimum just like a joystick

s(top)   Neutralizes the arrow key "joystick"

Revision
2013-11-06 KJ First version
2015-03-05 KJ Added queue_size to rospy.Publisher calls (Indigo compatiblity)
2015-03-19 KJ Switched from Bool to BoolStamped messages
2015-09-14 KJ Changed automode message type from BoolStamped to IntStamped
              and default topic name to /fmPlan/automode
2015-10-02 KJ Seperated the remote control output from the mission planner.
              This component is now converting the keyboard to a standardized
              remote control topic.
"""

import rospy
from std_msgs.msg import Char
from msgs.msg import RemoteControl

class ROSnode():
	def __init__(self):
		rospy.loginfo(rospy.get_name() + ": Start")
		# static parameters
		self.update_rate = 20 # Hz

		# keyboard interface
		self.KEY_ESC = 27
		self.KEY_SECOND = 91
		self.KEY_SPACE = 32
		self.KEY_a = 97
		self.KEY_e = 101
		self.KEY_r = 114
		self.KEY_s = 115
		self.KEY_ARROW_UP = 65
		self.KEY_ARROW_DOWN = 66
		self.KEY_ARROW_RIGHT = 67
		self.KEY_ARROW_LEFT = 68
		self.esc_key = False
		self.second_key = False
		self.up_down = 0
		self.left_right = 0
		self.udlr_step = 10
		self.udlr_max = 100

		# get topic names
		topic_kbd = rospy.get_param("~keyboard_sub", "/fmHMI/keyboard")
		topic_rc = rospy.get_param("~remote_control_pub",'/fmHMI/remote_control')

		# setup topic publishers
		self.rc_pub = rospy.Publisher(topic_rc, RemoteControl, queue_size=0)
		self.rc_msg = RemoteControl()
		self.rc_msg.remote_connected = True
		self.rc_msg.emergency_stop = False
		self.rc_msg.deadman_state = False
		self.rc_msg.switches = 0
		self.rc_msg.buttons = 0
		self.rc_msg.analog_button_a = 0 
		self.rc_msg.analog_button_b = 0
		self.rc_msg.analog_button_c = 0
		self.rc_msg.analog_button_d = 0
		self.rc_msg.digital_joy_a = 0
		self.rc_msg.digital_joy_b = 0
		self.rc_msg.analog_joy_a_up_down = 0
		self.rc_msg.analog_joy_a_left_right = 0
		self.rc_msg.analog_joy_b_up_down = 0
		self.rc_msg.analog_joy_b_left_right = 0
		self.rc_msg.rc_battery_percentage = 100
		self.rc_msg.rc_battery_low_warning = False
		self.rc_msg.rc_battery_low_alert = False

		# setup subscription topic callbacks
		rospy.Subscriber(topic_kbd, Char, self.on_keyboard_msg)

		# call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def on_keyboard_msg(self, msg):
		if self.esc_key == False:
			if msg.data == self.KEY_ESC: # the following key will be an escaped key
				self.esc_key = True
			elif msg.data == self.KEY_SPACE: # disable actuation
				self.rc_msg.deadman_state = False
				self.rc_msg.switches &= ~0x01
				self.up_down = 0
				self.left_right = 0
			elif msg.data == self.KEY_e: # enable actuation
				self.rc_msg.deadman_state = True
			elif msg.data == self.KEY_a:
				self.rc_msg.switches |= 0x01
				self.up_down = 0
				self.left_right = 0
			elif msg.data == self.KEY_r:
				self.rc_msg.switches &= ~0x01
			elif msg.data == self.KEY_s:
				self.up_down = 0
				self.left_right = 0
		else: # this key is a secondary key
			if msg.data == self.KEY_SECOND: # the following key will be a secondary key
				self.second_key = True
			elif self.second_key == True:
				self.esc_key = False
				self.second_key = False
				if msg.data == self.KEY_ARROW_UP:
					self.up_down += self.udlr_step
					if self.up_down > self.udlr_max:
						self.up_down = self.udlr_max
				elif msg.data == self.KEY_ARROW_DOWN:
					self.up_down -= self.udlr_step
					if self.up_down < -self.udlr_max:
						self.up_down = -self.udlr_max
				elif msg.data == self.KEY_ARROW_LEFT:
					self.left_right -= self.udlr_step
					if self.left_right < -self.udlr_max:
						self.left_right = -self.udlr_max
				elif msg.data == self.KEY_ARROW_RIGHT:
					self.left_right += self.udlr_step
					if self.left_right > self.udlr_max:
						self.left_right = self.udlr_max

	def publish_msg(self):
		self.rc_msg.header.stamp = rospy.get_rostime()
		self.rc_msg.analog_joy_a_up_down = self.up_down
		self.rc_msg.analog_joy_a_left_right = self.left_right
		self.rc_pub.publish(self.rc_msg)

	def updater(self):
		while not rospy.is_shutdown():
			self.publish_msg()		
			self.r.sleep() # go back to sleep

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('keyboard_remote_control')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ROSnode()
    except rospy.ROSInterruptException:
		pass

