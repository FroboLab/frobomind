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
This mission file provides a standard remote control output based on input from
a Nintendo Wiimote remote control.

Revision
2013-11-14 KJ First version
2015-10-03 KJ Seperated the remote control output from the mission planner.
              This component is now converting the wiimoote inpt to a
              standardized remote control topic.
"""

# Python imports
from math import pi

# ROS imports
import rospy
from std_msgs.msg import Char
from sensor_msgs.msg import Joy

# FroboMind imports
from msgs.msg import RemoteControl

class ROSnode():
	def __init__(self):
		rospy.loginfo(rospy.get_name() + ": Start")

		# wiimote
		self.button_a_prev = False
		self.switch_1_state = False
		self.pi_half = pi/2.0
		self.pitch_roll_min = 7.0/180.0*pi
		self.pitch_roll_max = 70.0/180.0*pi
		self.pitch_roll_scale = self.pitch_roll_min/(self.pitch_roll_max-self.pitch_roll_min)
		self.udlr_max = 100
		self.pitch_prev = 0.0
		self.roll_prev = 0.0
		self.wii_msg_tout = 0.0
		self.wii_msg_ok_delay = 0.25
		self.wii_pitch_roll_tout = 0.0
		self.wii_pitch_roll_ok_delay = 3.0

		# get parameters
		self.update_rate = 20 # Hz

		# get topic names
		topic_joy = rospy.get_param("~wiimote_sub",'/fmLib/joy')
		topic_rc = rospy.get_param("~remote_control_pub",'/fmHMI/remote_control')

		# setup topic publishers
		self.rc_pub = rospy.Publisher(topic_rc, RemoteControl, queue_size=0)
		self.rc_msg = RemoteControl()
		self.rc_msg.remote_connected = False
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
		rospy.Subscriber(topic_joy, Joy, self.on_joy_msg)

		# call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def on_joy_msg(self, msg):
		self.wii_msg_tout = rospy.get_time() + self.wii_msg_ok_delay
		self.rc_msg.remote_connected = True

		# determine pitch and roll
		pitch = float(msg.axes[1])*pi/20.0
		roll = float(msg.axes[0])*pi/20.0

		# check for movement
		if pitch != self.pitch_prev or roll != self.roll_prev:
			self.wii_pitch_roll_tout = rospy.get_time() + self.wii_pitch_roll_ok_delay
			self.pitch_prev = pitch
			self.roll_prev = roll

		# limit to self.pitch_roll_max
		if abs(pitch) < self.pitch_roll_min:
			pitch = 0.0
		elif pitch > self.pitch_roll_max:
			pitch = self.pitch_roll_max
		elif pitch < -self.pitch_roll_max:
			pitch = -self.pitch_roll_max

		if abs(roll) < self.pitch_roll_min:
			roll = 0.0
		elif roll > self.pitch_roll_max:
			roll = self.pitch_roll_max
		elif roll < -self.pitch_roll_max:
			roll = -self.pitch_roll_max

		# scale factor to accomodate deadband (within self.pitch_roll_min) when held approximately level
		if pitch > 0:
			pitch -= (self.pitch_roll_max-pitch)*self.pitch_roll_scale
		elif pitch < 0:
			pitch += (self.pitch_roll_max+pitch)*self.pitch_roll_scale

		if roll > 0:
			roll -= (self.pitch_roll_max-roll)*self.pitch_roll_scale
		elif roll < 0:
			roll += (self.pitch_roll_max+roll)*self.pitch_roll_scale

		# convert to [-100;100]
		self.rc_msg.analog_joy_a_left_right = roll*self.udlr_max/self.pitch_roll_max
		self.rc_msg.analog_joy_a_up_down = pitch*self.udlr_max/self.pitch_roll_max

		# handle deadman button
		button_b_state = (int(msg.buttons[3]) != 0) # button B
		if button_b_state == True and rospy.get_time() < self.wii_pitch_roll_tout:
			self.rc_msg.deadman_state = True
		else:
			self.rc_msg.deadman_state = False

		# handle switch #1 (modified button A)
		button_a = (int(msg.buttons[2]) != 0) # button A
		if button_a != self.button_a_prev:
			self.button_a_prev = button_a
			if button_a == True:
				self.switch_1_state = not(self.switch_1_state)

		# handle all other buttons
		if self.switch_1_state == True:
			self.rc_msg.switches |= 0x01
		else:
			self.rc_msg.switches &= ~0x01

		if int(msg.buttons[10]) != 0: # Home button
			self.rc_msg.buttons |= 0x01
		else:
			self.rc_msg.buttons &= ~0x01
		if int(msg.buttons[5]) != 0: # button -
			self.rc_msg.buttons |= 0x02
		else:
			self.rc_msg.buttons &= ~0x02
		if int(msg.buttons[4]) != 0: # button +
			self.rc_msg.buttons |= 0x04
		else:
			self.rc_msg.buttons &= ~0x04
		if int(msg.buttons[0]) != 0: # button 1
			self.rc_msg.buttons |= 0x08
		else:
			self.rc_msg.buttons &= ~0x08
		if int(msg.buttons[1]) != 0: # button 2
			self.rc_msg.buttons |= 0x010
		else:
			self.rc_msg.buttons &= ~0x010

		if int(msg.buttons[8]) != 0: # Up
			self.rc_msg.digital_joy_a |= 0x01
		else:
			self.rc_msg.digital_joy_a &= ~0x01
		if int(msg.buttons[9]) != 0: # Down
			self.rc_msg.digital_joy_a |= 0x02
		else:
			self.rc_msg.digital_joy_a &= ~0x02
		if int(msg.buttons[6]) != 0: # Left
			self.rc_msg.digital_joy_a |= 0x04
		else:
			self.rc_msg.digital_joy_a &= ~0x04
		if int(msg.buttons[7]) != 0: # Right
			self.rc_msg.digital_joy_a |= 0x08
		else:
			self.rc_msg.digital_joy_a &= ~0x08

	def publish_msg(self):
		self.rc_msg.header.stamp = rospy.get_rostime()
		self.rc_pub.publish(self.rc_msg)

	def updater(self):
		while not rospy.is_shutdown():
			if rospy.get_time() > self.wii_msg_tout:
				self.rc_msg.deadman_state = False
				self.rc_msg.remote_connected = False
			elif rospy.get_time() > self.wii_pitch_roll_tout:
				self.rc_msg.deadman_state = False
			self.publish_msg()		
			self.r.sleep() # go back to sleep

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('wiimote_remote_control')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ROSnode()
    except rospy.ROSInterruptException:
		pass

