#!/usr/bin/env python
#/****************************************************************************
# FroboMind Print navigation status
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

Revision
2013-12-04 KJ First version
"""

# ROS imports
import rospy
from msgs.msg import waypoint_navigation_status
from math import pi

class ros_node():
	def __init__(self):
		# constants
		self.rad_to_deg = 180.0/pi
		self.deg_to_rad = pi/180.0

		# parameters
		update_frequency = 10

		# state variable initialization
		self.state_prev = -1
		self.mode_prev = -1

		# Get parameters
		wptnav_status_topic = rospy.get_param("~wptnav_status_sub",'/fmInformation/wptnav_status')

		# Setup subscription topic callbacks
		rospy.Subscriber(wptnav_status_topic, waypoint_navigation_status, self.on_wptnav_status_topic)

		# Call updater function
		self.r = rospy.Rate(update_frequency) # set updater frequency
		self.updater()

	def time_stamp (self, stamp):
		secs = stamp.secs
		msecs = int(stamp.nsecs/1000000.0+0.5)
		if msecs == 1000:
			secs += 1 
			msecs = 0		
		return '%d.%03d' % (secs, msecs)

	# handle incoming waypoint navigation status messages
	def on_wptnav_status_topic(self, msg):
		if msg.state == 1 and msg.mode != 0:
			print "%s  dist %6.2f bearing %5.1f   t_dist %5.2f t_head_err %5.1f ab_dist %.2f   lin_v %5.2f ang_v %5.2f" % (self.time_stamp(msg.header.stamp), msg.distance_to_b, msg.bearing_to_b*self.rad_to_deg, msg.target_distance, msg.target_heading_err*self.rad_to_deg, msg.distance_to_ab_line, msg.linear_speed, msg.angular_speed)

		if msg.state != self.state_prev:
			self.state_prev = msg.state
			if msg.state == 0:
				print '%s  Idle state' % self.time_stamp(msg.header.stamp)
			elif msg.state == 1:
				print '%s  Navigation state' % self.time_stamp(msg.header.stamp)
			elif msg.state == 2:
				print '%s  Wait state' % self.time_stamp(msg.header.stamp)

		if msg.mode != self.mode_prev:
			self.mode_prev = msg.mode
			if msg.mode == 0:
				print '%s  Standby mode' % self.time_stamp(msg.header.stamp)
			elif msg.mode == 1:
				print '%s  Drive mode' % self.time_stamp(msg.header.stamp)
			elif msg.mode == 2:
				print '%s  Turn mode' % self.time_stamp(msg.header.stamp)
		

	# update loop
	def updater(self):
		while not rospy.is_shutdown():
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('print_navigation_status_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ros_node()
    except rospy.ROSInterruptException: pass

