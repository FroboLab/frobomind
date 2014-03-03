#!/usr/bin/env python
#/****************************************************************************
# FroboMind plot velocity node
# Copyright (c) 2014, Kjeld Jensen <kjeld@frobomind.org>
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
2014-03-03 KJ First version
"""

import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped
import matplotlib.pyplot as plt
#from pylab import plot, subplot, axis, grid, title, xlabel, ylabel, ylim, draw
from pylab import *

class ROSnode():
	def __init__(self):
		self.samples = 250
		self.vel_zero_max = 10

		# Get parameters
		self.update_frequency = rospy.get_param("~update_frequency", 10.0)
		self.title = rospy.get_param("~plot_title", "Velocity")
		self.max_lin_vel = rospy.get_param("~max_linear_velocity",1.0) # [m/s]
 		self.max_ang_vel = rospy.get_param("~max_angular_velocity",3.14) # [rad/s]
		self.samples = rospy.get_param("~samples", 200)
 
		# setup plot
		self.plot_vel_lin = [0]*self.samples
		self.plot_vel_ang = [0]*self.samples
		self.vel_zero_cnt = 0

		ion()
		subplot (211)
		self.p_vel_lin, = plot(self.plot_vel_lin, "red")
		ylim([-self.max_lin_vel, self.max_lin_vel])
		ylabel('Linear [m/s]')
		grid (True)
		title (self.title)
		subplot (212)
		self.p_vel_ang, = plot(self.plot_vel_ang, "red")
		ylim([-self.max_ang_vel, self.max_ang_vel])
		ylabel('Angular [rad/s]')
		xlabel('Samples')
		grid (True)
		draw() # redraw plot

		# Get topic names
		vel_topic = rospy.get_param("~velocity_sub",'/fmCommand/cmd_vel')

		# Setup subscription topic callbacks
		rospy.Subscriber(vel_topic, TwistStamped, self.on_vel_topic)

		# Call updater function
		self.r = rospy.Rate(self.update_frequency)
		self.updater()

	# handle incoming propulsion_feedback messages
	def on_vel_topic(self, msg):
		# check how many times the feedback has been all zeros
		if msg.twist.linear.x == 0.0 and msg.twist.angular.z == 0.0:
			self.vel_zero_cnt += 1
		else:
			self.vel_zero_cnt = 0

		# only update if within the self.vel_zero_max value
		if self.vel_zero_cnt <= self.vel_zero_max:
			self.plot_vel_lin.append (msg.twist.linear.x)
			del self.plot_vel_lin[0]
			self.plot_vel_ang.append (msg.twist.angular.z)
			del self.plot_vel_ang[0]

	# update loop
	def updater(self):
		while not rospy.is_shutdown():
			# replace plot y data
			self.p_vel_lin.set_ydata (self.plot_vel_lin)
			self.p_vel_ang.set_ydata (self.plot_vel_ang)
			draw() # redraw plot
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('plot_vel_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ROSnode()
    except rospy.ROSInterruptException: pass

