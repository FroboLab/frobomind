#!/usr/bin/env python
#/****************************************************************************
# FroboMind Plot lidar node
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
2013-12-05 KJ First version
"""

import rospy
#import numpy as np
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from pylab import plot, subplot, axis, grid, title, xlabel, ylabel, xlim, ylim, draw, ion
from math import sin, cos, pi

class ros_node():
	def __init__(self):
		# define local parameters
		self.lateral_max = 0.4 # [m]
		
		# Get launch parameters
		self.update_frequency = rospy.get_param("~update_frequency", 10.0)

		# initialize class variables
		self.new_data = False
		self.first_plot = True
		self.laser = [[0,0]]

		ion() # enable interactive mode

		# Get topic names
		laser_topic = rospy.get_param("~lidar_sub",'/fmSensors/laser_msg')
		# Setup subscription topic callbacks
		rospy.Subscriber(laser_topic, LaserScan, self.on_laser_topic)

		# Call updater function
		self.r = rospy.Rate(self.update_frequency)
		self.updater()

	# handle incoming laser messages
	def on_laser_topic(self, msg):
		self.new_data = True
		#print msg.angle_min, msg.angle_increment, msg.ranges[0]
		self.laser = []
		theta = msg.angle_min
		for i in xrange(len(msg.ranges)):
			r = msg.ranges[i]
			self.laser.append([r*sin(theta),r*cos(theta)]) # polar to cartesian coordinates
			theta += msg.angle_increment

	# update loop
	def updater(self):
		while not rospy.is_shutdown():
			if self.new_data:
				self.laserT = zip (*self.laser)
				if not self.first_plot:
					self.laser_plot.set_xdata (self.laserT[0])
					self.laser_plot.set_ydata (self.laserT[1])
					draw() # redraw plot
				else:
					self.first_plot = False
					self.laser_plot, = plot(self.laserT[0], self.laserT[1], "r.")
					xlim([-self.lateral_max, self.lateral_max])
					axis ('equal')					
					#ylim([-self.max_vel, self.max_vel])
					#ylabel('Velocity')
					#grid (True)
					title ('Laser scan')
					draw()	
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('plot_laser_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ros_node()
    except rospy.ROSInterruptException: pass

