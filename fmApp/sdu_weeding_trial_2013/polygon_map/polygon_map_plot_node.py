#!/usr/bin/env python
#/****************************************************************************
# Polygon Map
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
2013-06-10 KJ First version
"""

# imports
import rospy
from nav_msgs.msg import Odometry
from msgs.msg import IntStamped
from polygon_map_plot import polygon_map_plot
import csv

class PolygonMapPlotNode():
	def __init__(self):
		# defines
		rospy.loginfo(rospy.get_name() + ": Start")

		# get parameters
		self.update_rate = rospy.get_param("~update_rate", "5") # update frequency [Hz]
		self.offset_e = rospy.get_param("~easting_offset",0.0) # [m]
		self.offset_n = rospy.get_param("~northing_offset",0.0) # [m]
		self.trkpt_threshold = rospy.get_param("~trackpoint_threshold",0.1) # [m]
		map_title = rospy.get_param("~map_title", "Track")
		map_window_size = rospy.get_param("~map_window_size",5.0) # [inches]

		# initialize the polygon map
		self.polyplot = polygon_map_plot(map_title, map_window_size, self.offset_e, self.offset_n)

		# import polygons 
		file = open('polygon_map.txt', 'r')
		file_content = csv.reader(file, delimiter='\t')
		for name,e1,n1,e2,n2,e3,n3,e4,n4 in file_content:
			polygon = [[float(e1),float(n1)],[float(e2),float(n2)],[float(e3),float(n3)],[float(e4),float(n4)]]
			self.polyplot.add_polygon (polygon)
		file.close()
		rospy.loginfo(rospy.get_name() + ": Loaded %ld polygons" % self.polyplot.poly_total)

		# get topic names
		self.pose_topic = rospy.get_param("~pose_sub",'/fmKnowledge/pose')
		self.polygon_map_topic = rospy.get_param("~polygon_map_pub",'/fmKnowledge/polygon_map')

		# setup subscription topic callbacks
		rospy.Subscriber(self.pose_topic, Odometry, self.on_pose_message)
		rospy.Subscriber(self.polygon_map_topic, IntStamped, self.on_polygon_map_message)

		# call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def on_pose_message(self, msg):
		self.polyplot.update_pos (msg.pose.pose.position.x, msg.pose.pose.position.y)

	def on_polygon_map_message(self, msg):
		if msg.data > 0:
			self.polyplot.draw_polygon_within (msg.data-1)
		else:
			self.polyplot.draw_polygon_outside (-(msg.data+1))	

	def updater(self):
		while not rospy.is_shutdown():
			self.polyplot.update_map_plot()
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('polygon_map_plot_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = PolygonMapPlotNode()
    except rospy.ROSInterruptException:
		pass

