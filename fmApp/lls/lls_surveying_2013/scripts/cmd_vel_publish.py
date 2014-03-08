#!/usr/bin/env python
#/****************************************************************************
# cmd_vel_publish
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
Publish a defined cmd_vel along with a deadman signal at a defined interval. 

2013-11-01 KJ First version
"""

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, TwistStamped
from geometry_msgs.msg import Vector3

topic_deadman = rospy.get_param("~deadman_pub",'/fmCommand/deadman')
topic_cmd_vel = rospy.get_param("~cmd_vel_pub",'/fmCommand/cmd_vel')
update_interval = rospy.get_param("~update_interval", 0.1) # [s]
vel_lin = rospy.get_param("~linear_velocity", 0.5) # [m/s]
vel_ang = rospy.get_param("~angular_velocity", 0.0) # [rad/s]

rospy.init_node('cmd_vel_publish')
deadman_pub = rospy.Publisher(topic_deadman, Bool)
cmd_vel_pub = rospy.Publisher(topic_cmd_vel, TwistStamped)
cmd_vel_msg = TwistStamped()
deadman_msg = Bool()

rospy.logwarn (rospy.get_name() + ': Linear velocity  %.2f [m/s]' % (vel_lin))
rospy.logwarn (rospy.get_name() + ': Angular velocity %.2f [rad/s]' % (vel_ang))

while not rospy.is_shutdown():
	deadman_msg.data = True
	deadman_pub.publish(deadman_msg)

	cmd_vel_msg.twist.linear.x = vel_lin
	cmd_vel_msg.twist.angular.z = vel_ang
	cmd_vel_pub.publish(cmd_vel_msg)

	rospy.sleep(update_interval)

