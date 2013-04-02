#!/usr/bin/env python
#/****************************************************************************
# FroboMind show_map_node
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
# ROS imports
import rospy
from nav_msgs.msg import Odometry

# Math & plot imports
import matplotlib.pyplot as plt
from pylab import ion, plot, axis, grid, title, xlabel, ylabel, draw

class ShowMap():
    """
    ShowMap logs odometry positions and show the track log in an interactive map plot.
    drawMapDetails() may be edited to draw static map details.
    """
    def __init__(self):
        # Get parameters
	self.map_title = rospy.get_param("~map_title", "Track")
	self.map_window_size = rospy.get_param("~map_window_size",5.0) # [inches]
	self.trkpt_threshold = rospy.get_param("~trackpoint_threshold",0.1) # [m]
        rospy.loginfo(rospy.get_name() + " Trackpoint threshold: %.3f m" % (self.trkpt_threshold))
 
	# Get topic names
        self.odom_topic = rospy.get_param("~odom_sub",'/fmInformation/odom')

        # Setup subscription topic callbacks
        rospy.Subscriber(self.odom_topic, Odometry, self.onOdomTopic)
        
	# Initialize map
	self.origoX = 0
	self.origoY = 0
	self.track = [[0,0]]

	ion() # turn interaction mode on
	plt.figure(num=1, figsize=(self.map_window_size, self.map_window_size), dpi=80, facecolor='w', edgecolor='w')
	title (self.map_title)
	xlabel('Easting [m]')
	ylabel('Northing [m]')
	axis('equal')
	grid (True)
	self.drawMapDetails()

	# Call updater function
	self.r = rospy.Rate(1) # set updater frequency
	self.updater()

    def drawMapDetails(self):
	poly = [[-1.9,-2],[2,-1.9],[1.9,2],[-2,1.9]] # define polygon (just for testing)
	for i in range(len(poly)):
	    poly[i][0] -= self.origoX
	    poly[i][1] -= self.origoY
	poly.append(list(poly[0])) # without a typecast this will create a reference, not a copy
	polyT = zip (*poly)
	plt.figure(1)
	polyPlt = plot(polyT[0], polyT[1],'g')
	draw()

    def onOdomTopic(self,msg):
	# rospy.loginfo(rospy.get_name() + " Position: %.3f %.3f" % (msg.pose.pose.position.x,msg.pose.pose.position.y))
	x = msg.pose.pose.position.x - self.origoX
	y = msg.pose.pose.position.y - self.origoY
	if (abs(x-self.track[-1][0]) > self.trkpt_threshold or abs(y-self.track[-1][1]) > self.trkpt_threshold):
	 	self.track.append([x, y])

    def updater(self):
	while not rospy.is_shutdown():
		# Update map
		trackT = zip (*self.track)
		plt.figure(1)
		trackPlt = plot(trackT[0],trackT[1],'r')
		draw()
		self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('show_map_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ShowMap()
    except rospy.ROSInterruptException: pass

