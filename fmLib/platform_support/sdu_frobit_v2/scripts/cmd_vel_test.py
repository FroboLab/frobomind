#!/usr/bin/env python
#/****************************************************************************
# cmd_vel publisher example
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
This example publishes a cmd_vel message to control the linear and angular
speed of a robot.
"""
# imports
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# parameters
topic_cmd_vel = '/fmCommand/cmd_vel'
update_interval = 0.1 # [s]
vel_lin = 1.0 # [m/s]
vel_ang = 0.0 # [rad/s]

# launch node and create a publisher
rospy.init_node('cmd_vel_test')
pub = rospy.Publisher(topic_cmd_vel, Twist)

# loop until shutdown
while not rospy.is_shutdown():

	# publish the defined linear and angular velocity
	pub.publish(Twist(Vector3(vel_lin, 0, 0), Vector3(0, 0, vel_ang)))
	
	# sleep the defined interval
	rospy.sleep(update_interval)
 

