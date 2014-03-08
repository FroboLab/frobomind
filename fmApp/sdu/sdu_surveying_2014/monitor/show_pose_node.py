#!/usr/bin/env python
#/****************************************************************************
# show_pose
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
Prints the current pose at a defined interval. 

2013-12-05 KJ First version
"""

import rospy
from math import atan2, asin, pi
from nav_msgs.msg import Odometry

rad_to_deg = 180.0/pi
new_data = False

pose_x = 0.0
pose_y = 0.0
pose_z = 0.0
yaw = 0.0
pitch = 0.0
roll = 0.0

def on_pose_topic(msg):
	global pose_x, pose_y, pose_z, yaw, pitch, roll, new_data
	new_data = True

	# save x,y,z
	pose_x = msg.pose.pose.position.x
	pose_y = msg.pose.pose.position.y
	pose_z = msg.pose.pose.position.z

	# extract yaw, pitch and roll from the quaternion
	qx = msg.pose.pose.orientation.x
	qy = msg.pose.pose.orientation.y
	qz = msg.pose.pose.orientation.z
	qw = msg.pose.pose.orientation.w
	sqx = qx**2
	sqy = qy**2
	sqz = qz**2
	sqw = qw**2
	yaw = atan2(2*(qx*qy + qw*qz), sqw + sqx - sqy - sqz)
	pitch = asin((2*qy*qw) - (2*qx*qz))
	roll = atan2((2*qy*qz) + (2*qx*qw), sqw + sqz - sqy - sqx)
	
# init ros node
rospy.init_node('show_pose')

# read launch parameters
update_interval = rospy.get_param("~update_interval", 0.1) # [s]
topic_pose = rospy.get_param("~pose_sub",'/fmKnowledge/pose')

# set up subscribers
rospy.Subscriber(topic_pose, Odometry, on_pose_topic)

# main loop
while not rospy.is_shutdown():
	# do stuff
	if new_data:
		new_data = False	
	 	print 'Pose x: %.2f y: %.2f z: %.2f yaw: %4.1f pitch: %4.1f roll: %4.1f' % (pose_x, pose_y, pose_x, yaw*rad_to_deg, pitch*rad_to_deg, roll*rad_to_deg)

	# go back to sleep
	rospy.sleep(update_interval)

