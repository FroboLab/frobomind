#!/usr/bin/env python
#/****************************************************************************
# show_pose_2d
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

2013-11-01 KJ First version
"""

import rospy
from math import atan2
from nav_msgs.msg import Odometry

pose_yaw = 0.0
pose_x = 0.0
pose_y = 0.0

def on_pose_topic(msg):
	pose_x = msg.pose.pose.position.x
	pose_y = msg.pose.pose.position.y
	qx = msg.pose.pose.orientation.x
	qy = msg.pose.pose.orientation.y
	qz = msg.pose.pose.orientation.z
	qw = msg.pose.pose.orientation.w
	pose_yaw = atan2(2*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)

topic_pose = rospy.get_param("~pose_sub",'/fmKnowledge/pose')
update_interval = rospy.get_param("~update_interval", 0.1) # [s]

rospy.init_node('show_pose_2d')
rospy.Subscriber(pose_topic, Odometry, on_pose_topic)

while not rospy.is_shutdown():
 	print 'Pose x: %.2f y: %.2f yaw: %.1f' % (pose_x, pose_y, pose_yaw)
	rospy.sleep(update_interval)

