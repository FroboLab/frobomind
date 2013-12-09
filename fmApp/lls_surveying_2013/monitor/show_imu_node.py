#!/usr/bin/env python
#/****************************************************************************
# show_imu
# Copyright (c) 2013, Kjeld Jensen <kjeld@frobomind.org>
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
Prints the current imu yaw, pitch, roll at a defined interval. 

2013-12-05 KJ First version
"""

import rospy
from math import atan2, asin, pi
from sensor_msgs.msg import Imu

rad_to_deg = 180.0/pi
new_data = False

yaw = 0.0
pitch = 0.0
roll = 0.0

def on_imu_topic(msg):
	global yaw, pitch, roll, new_data
	new_data = True

	# extract yaw, pitch and roll from the quaternion
	qx = msg.orientation.x
	qy = msg.orientation.y
	qz = msg.orientation.z
	qw = msg.orientation.w
	sqx = qx**2
	sqy = qy**2
	sqz = qz**2
	sqw = qw**2
	yaw = atan2(2*(qx*qy + qw*qz), sqw + sqx - sqy - sqz)
	pitch = asin((2*qy*qw) - (2*qx*qz))
	roll = atan2((2*qy*qz) + (2*qx*qw), sqw + sqz - sqy - sqx)
	
# init ros node
rospy.init_node('show_imu')

# read launch parameters
update_interval = rospy.get_param("~update_interval", 0.1) # [s]
topic_imu = rospy.get_param("~imu_sub",'/fmInformation/imu')

# set up subscribers
rospy.Subscriber(topic_imu, Imu, on_imu_topic)

# main loop
while not rospy.is_shutdown():
	# do stuff
	if new_data:
		new_data = False	
	 	print 'yaw: %6.1f pitch: %5.1f roll: %5.1f' % (yaw*rad_to_deg, pitch*rad_to_deg, roll*rad_to_deg)

	# go back to sleep
	rospy.sleep(update_interval)

