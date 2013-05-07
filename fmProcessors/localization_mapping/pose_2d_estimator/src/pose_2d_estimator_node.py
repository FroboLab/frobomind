#!/usr/bin/env python
#*****************************************************************************
# FroboMind Pose 2D Estimator Node
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
#*****************************************************************************
"""
This file wraps the FroboMind Pose 2D Estimator library into a ROS node.
Most documentation of the library is in pose_2d_estimator.py

Revision
2013-05-03 KJ First version
"""
# ROS imports
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class Pose2DEstimatorNode():
    """
    ShowMap logs odometry positions and show the track log in an interactive map plot.
    drawMapDetails() may be edited to draw static map details.
    """
	def __init__(self):

		# Get parameters
		self.robot_max_speed = rospy.get_param("~robot_max_speed", "2.0") # Robot maximum speed [m/s]

		self.tm_a = rospy.get_param("~transverse_mercator_a", "6378137.0") # Equatorial radius, default is generic for WGS-84 datum
		self.tm_f = rospy.get_param("~transverse_mercator_f", "1/298.257223563") # Flattening, default is generic for WGS-84 datum
		self.tm_fe = rospy.get_param("~transverse_mercator_false_easting", "500000.0") # False Easting, default is generic for UTM projection
		self.tm_scale = rospy.get_param("~transverse_mercator_scale_factor", "0.9996") # Scale Factor, default is generic for UTM projection
		self.tm_orglat = rospy.get_param("~transverse_mercator_origin_latitude", "0.0")  # Origin Latitude, default is generic for UTM projection
		self.tm_cmer = rospy.get_param("~transverse_mercator_central_meridian", "9.0 ")  # Central Meridian, default is UTM32
		self.tm_fn = rospy.get_param("~transverse_mercator_false_northing", "0.0")  # False northing, default is for UTM northern hemisphere
 
		# Get topic names
		self.odom_topic = rospy.get_param("~odom_sub",'/fmKnowledge/encoder_odom')
		self.imu_topic = rospy.get_param("~imu_sub",'/fmInformation/imu')
		self.gga_topic = rospy.get_param("~gga_sub",'/fmInformation/gpgga')
		self.pose_topic = rospy.get_param("~pose_pub",'/fmKnowledge/pose')

		# Setup subscription topic callbacks
		rospy.Subscriber(self.odom_topic, Odometry, self.on_odom_topic)
		rospy.Subscriber(self.imu_topic, Odometry, self.on_imu_topic)
		rospy.Subscriber(self.gga_topic, Odometry, self.on_gga_topic)
  
		# more stuff
		rospy.loginfo(rospy.get_name() + "Just started %d" % (1))
      
		# Call updater function
		self.r = rospy.Rate(1) # set updater frequency
		self.updater()

	def on_odom_topic(self,msg):
		# rospy.loginfo(rospy.get_name() + " Position: %.3f %.3f" % (msg.pose.pose.position.x,msg.pose.pose.position.y))
		x = msg.pose.pose.position.x - self.origoX
		y = msg.pose.pose.position.y - self.origoY
		if (abs(x-self.track[-1][0]) > self.trkpt_threshold or abs(y-self.track[-1][1]) > self.trkpt_threshold):
			self.track.append([x, y])

	def on_imu_topic(self,msg):
		pass

	def on_gga_topic(self,msg):
		pass

	def updater(self):
		while not rospy.is_shutdown():
			# do updating stuff

			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('pose_2d_estimator_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ose2DEstimatorNode()
    except rospy.ROSInterruptException: pass

