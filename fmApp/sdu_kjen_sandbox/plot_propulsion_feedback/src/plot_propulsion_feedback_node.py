#!/usr/bin/env python
#/****************************************************************************
# FroboMind Propulsion feedback node
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

Revision
2013-11-17 KJ First version
"""

import rospy
import numpy as np
from msgs.msg import PropulsionModuleFeedback
import matplotlib.pyplot as plt
#from pylab import plot, subplot, axis, grid, title, xlabel, ylabel, ylim, draw
from pylab import *

class propulsion_feedback_node():
	def __init__(self):
		self.measurements = 250
		self.feedback_zero_max = 10

		# Get parameters
		self.update_frequency = rospy.get_param("~update_frequency", 10.0)
		self.title = rospy.get_param("~plot_title", "Feedback")
		self.max_vel = rospy.get_param("~maximum_velocity",3.0) # [m/s]
 		self.max_thrust = rospy.get_param("~maximum_thrust",1024)

		# setup plot
		self.plot_vel = [0]*self.measurements
		self.plot_vel_set = [0]*self.measurements
		self.plot_thrust = [0]*self.measurements
		self.feedback_zero_cnt = 0

		ion()
		subplot (211)
		self.p_vel, = plot(self.plot_vel, "red")
		self.p_vel_set, = plot(self.plot_vel_set, "blue")
		ylim([-self.max_vel, self.max_vel])
		ylabel('Velocity')
		grid (True)
		title (self.title)
		subplot (212)
		self.p_thrust, = plot(self.plot_thrust)
		ylim([-self.max_thrust, self.max_thrust])
		ylabel('Thrust')
		grid (True)
		draw() # redraw plot

		# Get topic names
		prop_fb_topic = rospy.get_param("~propulsion_feedback_sub",'/fmInformation/wheel_feedback_right')

		# Setup subscription topic callbacks
		rospy.Subscriber(prop_fb_topic, PropulsionModuleFeedback, self.on_propulsion_fb_topic)

		# Call updater function
		self.r = rospy.Rate(self.update_frequency)
		self.updater()

	# handle incoming propulsion_feedback messages
	def on_propulsion_fb_topic(self, msg):
		# check how many times the feedback has been all zeros
		if msg.velocity == 0.0 and msg.velocity_setpoint == 0.0 and msg.thrust == 0.0:
			self.feedback_zero_cnt += 1
		else:
			self.feedback_zero_cnt = 0

		# only update if within the self.feedback_zero_max value
		if self.feedback_zero_cnt <= self.feedback_zero_max:
			self.plot_vel.append (msg.velocity)
			del self.plot_vel[0]
			self.plot_vel_set.append (msg.velocity_setpoint)
			del self.plot_vel_set[0]
			self.plot_thrust.append (msg.thrust)
			del self.plot_thrust[0]

	# update loop
	def updater(self):
		while not rospy.is_shutdown():
			# replace plot y data
			self.p_vel.set_ydata (self.plot_vel)
			self.p_vel_set.set_ydata (self.plot_vel_set)
			self.p_thrust.set_ydata (self.plot_thrust)
			draw() # redraw plot
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('propulsion_feedback_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = propulsion_feedback_node()
    except rospy.ROSInterruptException: pass

