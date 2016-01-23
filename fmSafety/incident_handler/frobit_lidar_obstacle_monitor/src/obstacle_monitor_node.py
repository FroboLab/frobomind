#!/usr/bin/env python
#/****************************************************************************
# FroboMind obstacle_monitor_node
# Copyright (c) 2015, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#	* Redistributions of source code must retain the above copyright
#	  notice, this list of conditions and the following disclaimer.
#	* Redistributions in binary form must reproduce the above copyright
#	  notice, this list of conditions and the following disclaimer in the
#	  documentation and/or other materials provided with the distribution.
#   * Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
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

2015-08-19 KJ First version
2015-09-24 KJ Added obstacle topic
"""

import rospy
from msgs.msg import IntStamped, IntArrayStamped

class ROSnode():
	def __init__(self):
		rospy.loginfo(rospy.get_name() + ": Start")
		# defines

		# static parameters
		self.update_rate = 20 # set update frequency [Hz]

		# get topic names
		self.topic_obstacle= rospy.get_param("~obstacle_sub",'/fmKnowledge/obstacle')
		self.topic_obstacle_safe = rospy.get_param("~obstacle_safe_pub",'/fmSafe/obstacle')

		# setup topic publishers
		self.obstacle_safe_pub = rospy.Publisher(self.topic_obstacle_safe, IntStamped, queue_size=0)
		self.obstacle_safe_msg = IntStamped()

		# setup topic subscribers
		rospy.Subscriber(self.topic_obstacle, IntArrayStamped, self.on_obstacle_msg)

		# call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def on_obstacle_msg(self, msg):
		# save current state and determine next timeout
		if msg.data[0] == 2 or msg.data[1] == 2:
			self.obstacle_safe_msg.data = 1
		else:
			self.obstacle_safe_msg.data = 0
		self.obstacle_safe_msg.header.stamp = rospy.get_rostime()
		self.obstacle_safe_pub.publish (self.obstacle_safe_msg)

	def updater(self):
		while not rospy.is_shutdown():
			self.obstacle_safe_msg.header.stamp = rospy.get_rostime()
			self.obstacle_safe_pub.publish (self.obstacle_safe_msg)

			# go back to sleep
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('obstacle_monitor')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ROSnode()
    except rospy.ROSInterruptException:
		pass

