#!/usr/bin/env python
#/****************************************************************************
# FroboMind real-time analyze node
# Copyright (c) 2014, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#	* Redistributions of source code must retain the above copyright
#	  notice, this list of conditions and the following disclaimer.
#	* Redistributions in binary form must reproduce the above copyright
#	  notice, this list of conditions and the following disclaimer in the
#	  documentation and/or other materials provided with the distribution.
#	* Neither the name FroboMind nor the
#	  names of its contributors may be used to endorse or promote products
#	  derived from this software without specific prior written permission.
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
This node utilizes an external microcontroller unit to determine the real-time
performance. Please see the component readme.txt for more information.

2014-02-21 KJ First version
"""

import rospy
from msgs.msg import IntStamped
import serial

class ROSnode():
	def __init__(self):
		rospy.loginfo(rospy.get_name() + ": Start")
		# defines
		self.c = '$'

		# static parameters
		self.update_rate = 100 # set update frequency [Hz]

		# get parameters
		self.device = rospy.get_param("~device", "/dev/ttyUSB0") #

		# get topic names
		self.topic_timing = rospy.get_param("~timing_pub",'/fmInformation/rt_timing')

		# setup topic publishers
		self.timing_pub = rospy.Publisher(self.topic_timing, IntStamped)
		self.timing_msg = IntStamped()

		# configure and open serial device
		ser_error = False
		try :
			self.ser = serial.Serial(self.device, 57600, timeout=0)
		except Exception as e:
			rospy.logerr(rospy.get_name() + ": Unable to open serial device: %s" % self.device)
			ser_error = True

		if ser_error == False:
			# call updater function
			self.r = rospy.Rate(self.update_rate)
			self.updater()

	def updater(self):
		while not rospy.is_shutdown():
			self.c = self.ser.read()		
			self.ser.write ('$') # don't put this before ser.read()
			if len(self.c) == 1:
				self.timing_msg.header.stamp = rospy.Time.now()
				self.timing_msg.data = ord(self.c) 
				self.timing_pub.publish(self.timing_msg)

			# go back to sleep
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('real_time_analyze')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ROSnode()
    except rospy.ROSInterruptException:
		pass

