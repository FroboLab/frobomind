#!/usr/bin/env python
#/****************************************************************************
# FroboMind publish cmd_vel node
# Copyright (c) 2016, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#	* Redistributions of source code must retain the above copyright
#	  notice, this list of conditions and the following disclaimer.
#	* Redistributions in binary form must reproduce the above copyright
#	  notice, this list of conditions and the following disclaimer in the
#	  documentation and/or other materials provided with the distribution.
#	* Neither the name of the copyright holder nor the names of its
#	contributors may be used to endorse or promote products derived from
#	this software without specific prior written permission.
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
This node publishes a cmd_vel command.

2016-03-16 KJ First version
"""

import rospy
from geometry_msgs.msg import TwistStamped

class ROSnode():
	def __init__(self):
		rospy.loginfo(rospy.get_name() + ": Start")

		# static parameters
		self.update_rate = 10 # set update frequency [Hz]
		self.linear_vel = 0.2 # [m/s]
		self.angular_vel = 0.1 # [rad/s]

		# get topic names
		self.topic_cmd_vel = rospy.get_param("~cmd_vel_pub",'/fmCommand/cmd_vel')

		# setup topic publishers
		self.cmd_vel_pub = rospy.Publisher(self.topic_cmd_vel, TwistStamped)
		self.cmd_vel_msg = TwistStamped()

		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def updater(self):
		while not rospy.is_shutdown():
			self.cmd_vel_msg.header.stamp = rospy.Time.now()
			self.cmd_vel_msg.twist.linear.x = self.linear_vel
			self.cmd_vel_msg.twist.angular.z = self.angular_vel
			self.cmd_vel_pub.publish(self.cmd_vel_msg)

			# go back to sleep
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('publish_cmd_vel')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ROSnode()
    except rospy.ROSInterruptException:
		pass

