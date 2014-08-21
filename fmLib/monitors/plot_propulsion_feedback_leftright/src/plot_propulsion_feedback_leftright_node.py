#!/usr/bin/env python
#/****************************************************************************
# FroboMind Propulsion feedback leftright node
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

Revision
2013-11-17 KJ First version
2014-02-25 KJ Updated to support both left and right wheel
"""

import rospy
import numpy as np
from msgs.msg import PropulsionModuleFeedback
import matplotlib.pyplot as plt
#from pylab import plot, subplot, axis, grid, title, xlabel, ylabel, ylim, draw
from pylab import *

class propulsion_feedback_node():
	def __init__(self):
		self.feedback_zero_max = 10

		# Get parameters
		self.update_frequency = rospy.get_param("~update_frequency", 10.0)
		self.title = rospy.get_param("~plot_title", "Feedback")
		self.samples = rospy.get_param("~samples", 250)
		self.max_vel = rospy.get_param("~maximum_velocity",3.0) # [m/s]
 		self.max_thrust = rospy.get_param("~maximum_thrust",1024)

		# setup plot
		self.plot_vel_left = [0]*self.samples
		self.plot_vel_set_left = [0]*self.samples
		self.plot_thrust_left = [0]*self.samples
		self.feedback_zero_cnt_left = 0

		self.plot_vel_right = [0]*self.samples
		self.plot_vel_set_right = [0]*self.samples
		self.plot_thrust_right = [0]*self.samples
		self.feedback_zero_cnt_right = 0

		ion()
		subplot (311)
		self.p_vel_left, = plot(self.plot_vel_left, "red")
		self.p_vel_set_left, = plot(self.plot_vel_set_left, "black")
		ylim([-self.max_vel, self.max_vel])
		ylabel('V left')
		grid (True)
		title (self.title)
		subplot (312)
		self.p_vel_right, = plot(self.plot_vel_right, "green")
		self.p_vel_set_right, = plot(self.plot_vel_set_right, "black")
		ylim([-self.max_vel, self.max_vel])
		ylabel('V right')
		grid (True)
		subplot (313)
		self.p_thrust_left, = plot(self.plot_thrust_left, 'red')
		self.p_thrust_right, = plot(self.plot_thrust_right, 'green')
		ylim([-self.max_thrust, self.max_thrust])
		ylabel('Thrust')
		grid (True)
		draw() # redraw plot

		# Get topic names
		prop_fb_left_topic = rospy.get_param("~propulsion_feedback_left_sub",'/fmInformation/wheel_feedback_left')
		prop_fb_right_topic = rospy.get_param("~propulsion_feedback_right_sub",'/fmInformation/wheel_feedback_right')

		# Setup subscription topic callbacks
		rospy.Subscriber(prop_fb_left_topic, PropulsionModuleFeedback, self.on_propulsion_fb_left_topic)
		rospy.Subscriber(prop_fb_right_topic, PropulsionModuleFeedback, self.on_propulsion_fb_right_topic)

		# Call updater function
		self.r = rospy.Rate(self.update_frequency)
		self.updater()

	# handle incoming propulsion_feedback messages
	def on_propulsion_fb_left_topic(self, msg):
		# check how many times the feedback has been all zeros
		if msg.velocity == 0.0 and msg.velocity_setpoint == 0.0 and msg.thrust == 0.0:
			self.feedback_zero_cnt_left += 1
		else:
			self.feedback_zero_cnt_left = 0

		# only update if within the self.feedback_zero_max value
		if self.feedback_zero_cnt_left <= self.feedback_zero_max:
			self.plot_vel_left.append (msg.velocity)
			del self.plot_vel_left[0]
			self.plot_vel_set_left.append (msg.velocity_setpoint)
			del self.plot_vel_set_left[0]
			self.plot_thrust_left.append (msg.thrust)
			del self.plot_thrust_left[0]

	# handle incoming propulsion_feedback messages
	def on_propulsion_fb_right_topic(self, msg):
		# check how many times the feedback has been all zeros
		if msg.velocity == 0.0 and msg.velocity_setpoint == 0.0 and msg.thrust == 0.0:
			self.feedback_zero_cnt_right += 1
		else:
			self.feedback_zero_cnt_right = 0

		# only update if within the self.feedback_zero_max value
		if self.feedback_zero_cnt_right <= self.feedback_zero_max:
			self.plot_vel_right.append (msg.velocity)
			del self.plot_vel_right[0]
			self.plot_vel_set_right.append (msg.velocity_setpoint)
			del self.plot_vel_set_right[0]
			self.plot_thrust_right.append (msg.thrust)
			del self.plot_thrust_right[0]

	# update loop
	def updater(self):
		while not rospy.is_shutdown():
			# replace plot y data
			self.p_vel_left.set_ydata (self.plot_vel_left)
			self.p_vel_set_left.set_ydata (self.plot_vel_set_left)
			self.p_thrust_left.set_ydata (self.plot_thrust_left)
			self.p_vel_right.set_ydata (self.plot_vel_right)
			self.p_vel_set_right.set_ydata (self.plot_vel_set_right)
			self.p_thrust_right.set_ydata (self.plot_thrust_right)
			draw() # redraw plot
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('propulsion_feedback_leftright_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = propulsion_feedback_node()
    except rospy.ROSInterruptException: pass

