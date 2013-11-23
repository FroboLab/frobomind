#!/usr/bin/env python
#/****************************************************************************
# Position log script
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
This file logs robot pose data while the robot is waiting at a waypoint.

Revision
2013-11-14 KJ First version
"""

import rospy
from msgs.msg import waypoint_navigation_status

class log_node():
	def __init__(self):
		self.update_rate = 20 # [Hz]

		# robot state
		self.STATE_IDLE = 0
		self.STATE_LOG = 1
		self.state = self.STATE_IDLE

		# read parameters

		# get topic names
		wptnav_status_topic = rospy.get_param("~keyboard_sub", "/fmData/wptnav_status")

		# setup subscription topic callbacks
		rospy.Subscriber(wptnav_status_topic, waypoint_navigation_status, self.on_wptnav_status_topic)

		# sall updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def on_wptnav_status_topic(self, msg):
		wptnav_wait = (msg.state == 2)
		if wptnav_wait and self.state == self.STATE_IDLE:
			self.begin_log()
		elif wptnav_wait == False and self.state == self.STATE_LOG
			self.end_log()				
	
	def begin_log (self)
		self.state = self.STATE_LOG

	def end_log (self)
		self.state = self.STATE_IDLE

	def updater(self):
		while not rospy.is_shutdown():
			self.r.sleep()

# main function.    
if __name__ == '__main__':
    # initialize the node and name it.
    rospy.init_node('position_log')

    # go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = log_node()
    except rospy.ROSInterruptException: pass


