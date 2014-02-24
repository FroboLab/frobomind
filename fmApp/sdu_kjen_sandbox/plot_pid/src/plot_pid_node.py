#!/usr/bin/env python
#/****************************************************************************
# FroboMind plot PID node
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
2014-02-24 KJ First version
"""

import rospy
import numpy as np
from msgs.msg import FloatArrayStamped
import matplotlib.pyplot as plt
from pylab import plot, subplot, axis, grid, title, xlabel, ylabel, ylim, draw, ion

class ROSnode():
	def __init__(self):
		self.pid_zero_max = 10
		self.pid_zero_cnt = 0

		# Get parameters
		self.update_rate = rospy.get_param("~update_rate", 10.0)
		self.title = rospy.get_param("~plot_title", "PID status")
		self.samples = rospy.get_param("~samples", 50)
		self.max_error = rospy.get_param("~max_error",1.0) # [m/s]
		self.max_out = rospy.get_param("~max_output",1.0) # [m/s]
 		self.max_pid = rospy.get_param("~max_pid",1.0)
		self.plot_feed_forward = rospy.get_param("~plot_feed_forward", "false")

		# setup plot
		self.e = [0]*self.samples
		self.o = [0]*self.samples
		self.p = [0]*self.samples
		self.i = [0]*self.samples
		self.d = [0]*self.samples
		if self.plot_feed_forward == True:
			self.f = [0]*self.samples

		ion()
		subplot (311)
		self.plt_e, = plot(self.e, 'black')
		ylim([-self.max_error, self.max_error])
		ylabel('Error')
		grid (True)
		title (self.title)
		subplot (312)
		self.plt_o, = plot(self.o, 'black')
		ylim([-self.max_out, self.max_out])
		ylabel('Output')
		grid (True)
		subplot (313)
		if self.plot_feed_forward == True:
			self.plt_f, = plot(self.f, 'black')
		self.plt_p, = plot(self.p, 'red')
		self.plt_i, = plot(self.i, 'green')
		self.plt_d, = plot(self.d, 'blue')
		ylim([-self.max_pid, self.max_pid])
		ylabel('PID')
		grid (True)
		draw() # redraw plot

		# Get topic names
		pid_topic = rospy.get_param("~pid_sub",'/fmInformation/pid')

		# Setup subscription topic callbacks
		rospy.Subscriber(pid_topic, FloatArrayStamped, self.on_pid_topic)

		# Call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	# handle incoming propulsion_feedback messages
	def on_pid_topic(self, msg):
		# check how many times the feedback has been all zeros
		if msg.data[0] == 0.0 and msg.data[1] == 0.0:
			self.pid_zero_cnt += 1
		else:
			self.pid_zero_cnt = 0

		# only update if within the self.feedback_zero_max value
		if self.pid_zero_cnt <= self.pid_zero_max:
			self.e.append (msg.data[0])
			del self.e[0]
			self.o.append (msg.data[1])
			del self.o[0]
			self.p.append (msg.data[2])
			del self.p[0]
			self.i.append (msg.data[3])
			del self.i[0]
			self.d.append (msg.data[4])
			del self.d[0]
			if self.plot_feed_forward == True:
				self.f.append (msg.data[5])
				del self.f[0]

	# update loop
	def updater(self):
		while not rospy.is_shutdown():
			# replace plot y data
			self.plt_e.set_ydata (self.e)
			self.plt_o.set_ydata (self.o)
			self.plt_p.set_ydata (self.p)
			self.plt_i.set_ydata (self.i)
			self.plt_d.set_ydata (self.d)
			if self.plot_feed_forward == True:
				self.plt_f.set_ydata (self.f)
			draw() # redraw plot
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('plot_pid_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ROSnode()
    except rospy.ROSInterruptException: pass

