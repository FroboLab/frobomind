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
from polygon_map import polygon_map
import csv

class PolygonMapNode():
	def __init__(self):
		rospy.loginfo(rospy.get_name() + ": Start")

		# get parameters
		self.update_rate = rospy.get_param("~update_rate", "10") # update frequency [Hz]
		self.polygons_per_update = rospy.get_param("~polygons_per_update", "100")
		self.nearby_threshold = rospy.get_param("~nearby_threshold", "5.0") # [m]
		rospy.loginfo(rospy.get_name() + ": Update rate: %ld Hz", self.update_rate)
		rospy.loginfo(rospy.get_name() + ": Polygons per update: %ld", self.polygons_per_update)
		rospy.loginfo(rospy.get_name() + ": Nearby threshold: %.3f m", self.nearby_threshold)

		# get topic names
		self.pose_topic = rospy.get_param("~pose_sub",'/fmKnowledge/pose')
		self.polygon_map_topic = rospy.get_param("~polygon_map_pub",'/fmKnowledge/polygon_map')

		# setup subscription topic callbacks
		rospy.Subscriber(self.pose_topic, Odometry, self.on_pose_message)

		# setup publish topics
		self.polygon_change_pub = rospy.Publisher(self.polygon_map_topic, IntStamped)
		self.polygon_change = IntStamped()

		# initialize the polygon map
		self.polymap = polygon_map()
		self.polymap.set_nearby_threshold (self.nearby_threshold) 
		self.polymap.set_polygons_per_update (self.polygons_per_update)

		# import polygons 
		file = open('polygon_map.txt', 'r')
		file_content = csv.reader(file, delimiter='\t')
		for name,e1,n1,e2,n2,e3,n3,e4,n4 in file_content:
			polygon = [[float(e1),float(n1)],[float(e2),float(n2)],[float(e3),float(n3)],[float(e4),float(n4)]]
			self.polymap.add_polygon (name, polygon)
		file.close()
		rospy.loginfo(rospy.get_name() + ": Loaded %ld polygons" % self.polymap.poly_total)

		# call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def on_pose_message(self, msg):
		self.polymap.update_pos (msg.pose.pose.position.x, msg.pose.pose.position.y)
	
	def publish_polygon_change_message(self, change):
		self.polygon_change.header.stamp = rospy.Time.now()
		self.polygon_change.data = change
		self.polygon_change_pub.publish (self.polygon_change)

	def updater(self):
		while not rospy.is_shutdown():
			time_ros = rospy.Time.now()
			time_secs = time_ros.secs + time_ros.nsecs*1e-9
			polygon_changes = self.polymap.update_map(time_secs)
			for i in xrange(len(polygon_changes)):
				self.publish_polygon_change_message (polygon_changes[i])
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('polygon_map_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = PolygonMapNode()
    except rospy.ROSInterruptException:
		pass

