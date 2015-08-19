#!/usr/bin/env python
#/****************************************************************************
# FroboMind basic_incident_handler_node
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
"""

import rospy
from msgs.msg import BoolStamped, IntStamped

class ROSnode():
	def __init__(self):
		rospy.loginfo(rospy.get_name() + ": Start")
		# defines

		# static parameters
		self.update_rate = 50 # set update frequency [Hz]

		# get parameters
		self.deadman_en = rospy.get_param("~deadman_enable", True)
		self.critfault_en = rospy.get_param("~critical_fault_enable", True) 
		self.deadman_timeout = rospy.get_param("~deadman_timeout", 0.050) # [s]
		self.critfault_timeout = rospy.get_param("~critical_fault_timeout", 0.050) # [s]

		# get topic names
		self.topic_deadman = rospy.get_param("~deadman_sub",'/fmSafe/deadman')
		self.topic_critical_fault = rospy.get_param("~critical_fault_sub",'/fmSafe/critical_fault')
		self.topic_act_en = rospy.get_param("~actuation_enable_pub",'/fmSafe/actuation_enable')

		# setup topic publishers
		self.act_en_pub = rospy.Publisher(self.topic_act_en, BoolStamped, queue_size=0)
		self.act_en_msg = BoolStamped()

		# setup topic subscribers
		self.deadman_state = False
		self.deadman_next_tout = 0.0
		rospy.Subscriber(self.topic_deadman, BoolStamped, self.on_deadman_msg)
		self.critfault_state = False
		self.critfault_next_tout = 0.0
		rospy.Subscriber(self.topic_critical_fault, IntStamped, self.on_critfault_msg)

		# call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def on_deadman_msg(self, msg):
		# save current state and determine next timeout
		self.deadman_state = msg.data
		self.deadman_next_tout = rospy.get_time() + self.deadman_timeout

	def on_critfault_msg(self, msg):
		# save current state and determine next timeout
		self.critfault_state = msg.data
		self.critfault_next_tout = rospy.get_time()  + self.critfault_timeout

	def updater(self):
		while not rospy.is_shutdown():
			# default is True
			prev_act_en = self.act_en_msg.data
			self.act_en_msg.data = True

			# cricital fault if true or too old
			if self.critfault_en == True:
				if self.critfault_state != 0 or self.critfault_next_tout < rospy.get_time():
					self.act_en_msg.data = False

			# deadman fault if false or too old
			if self.deadman_en == True:
				if self.deadman_state == False or self.deadman_next_tout < rospy.get_time():
					self.act_en_msg.data = False
					#print self.deadman_next_tout, rospy.get_time(), self.deadman_state

			# publish actuation_enable message
			self.act_en_msg.header.stamp = rospy.get_rostime()
			self.act_en_pub.publish (self.act_en_msg)

			if prev_act_en != self.act_en_msg.data:
				if self.act_en_msg.data == True:
					rospy.logwarn(rospy.get_name() + ": Enabling actuation")
				else:
					rospy.logwarn(rospy.get_name() + ": Disabling actuation")

			# go back to sleep
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('basic_incident_handler')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ROSnode()
    except rospy.ROSInterruptException:
		pass

