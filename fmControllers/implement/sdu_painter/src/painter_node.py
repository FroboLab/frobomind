#!/usr/bin/env python
#/****************************************************************************
# Paint marker interface
# Copyright (c) 2015, Mathias Neerup <manee12@student.sdu.dk>,
#                     Kjeld Jensen <kjeld@frobomind.org>
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
2014-09-23 KJ First version
2015-05-03 MMN Code migrated from FroboMind lighthouse to FroboScout2 paint implement
2015-06-30 KJ Fused two nodes into one and removed some bugs.

The launch parameter "paint_time" specifies the paint activity:
	paint_time = 0: The painter will paint whenever painter_cmd_sub is True.

	paint_time > 0: The painter will paint for the specified time [s] when
	painter_cmd_sub becomes True. A reset is performed when painter_cmd_sub
	becomes False

"""

# ROS imports
import rospy
from msgs.msg import nmea, BoolStamped

ST_PAINT_IDLE = 0
ST_PAINT_BEGIN = 1
ST_PAINT_STOP = 2

class ros_node():
	def __init__(self):
		# constants
		
		# parameters
		update_frequency = 5

		# state variable initialization
		self.painter_state = ST_PAINT_IDLE
		self.last_painter_state = ST_PAINT_IDLE
		self.last_state = False

		# Get parameters
		topic_implement_painter = rospy.get_param("~painter_cmd_sub",'/fmCommand/implement')
		topic_painter_rx = rospy.get_param("~nmea_from_painter_sub",'/fmData/nmea_from_painter')
		topic_painter_tx = rospy.get_param("~nmea_to_painter_pub",'/fmData/nmea_to_painter')
		self.paintertime = rospy.get_param("~paint_time",'0.5') # [s]
		if self.paintertime < 0.01:
			self.paint_timeout = False
		else:
			self.paint_timeout = True

		# define NMEA $pfrrs topic
		self.nmea_pfrrs = nmea()
		self.nmea_pfrrs.type = 'PFRRS'
		self.nmea_pfrrs.length = 1
		self.nmea_pfrrs.valid = True
		self.nmea_pfrrs.data.append(str(self.painter_state))

		# Setup subscription topic callbacks
		rospy.Subscriber(topic_implement_painter, BoolStamped, self.on_painter_msg)

		# setup topic publishers
		self.nmea_pub = rospy.Publisher(topic_painter_tx, nmea, queue_size=1)

		# Call updater function
		self.r = rospy.Rate(update_frequency) # set updater frequency
		self.updater()

	def on_painter_msg(self, msg):
		state = msg.data

		if state == True and self.last_state == False:
			# Start painter
			self.painter_state = ST_PAINT_BEGIN

			if self.paint_timeout == True:
				# Timer to keep spraying
				rospy.Timer(rospy.Duration(float(self.paintertime)), self.timer_stop_painter, True)

		elif state == False and self.last_state == True:
			if self.paint_timeout == False:
				self.painter_state = ST_PAINT_STOP

		self.last_state = state

	# On timer callback, stop painting
	def timer_stop_painter(self, event):
		self.painter_state = ST_PAINT_STOP

	def update_painter(self):
		# Pack the NMEA msg and publish it
		self.nmea_pfrrs.header.stamp = rospy.get_rostime()
		self.nmea_pfrrs.data[0] = str(self.painter_state)
		self.nmea_pub.publish (self.nmea_pfrrs)


		# Go to next state
		#if self.painter_state == ST_PAINT_BEGIN:
		#	self.painter_state = ST_PAINT_IDLE

		# Go to next state
		if self.painter_state == ST_PAINT_STOP:
			self.painter_state = ST_PAINT_IDLE

	# update loop
	def updater(self):
		while not rospy.is_shutdown():
			self.update_painter()
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('painter_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ros_node()
    except rospy.ROSInterruptException: pass

