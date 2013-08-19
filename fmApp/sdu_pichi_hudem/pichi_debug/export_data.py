#!/usr/bin/env python
#*****************************************************************************
# export_simulation_data
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
#*****************************************************************************
"""
This script extract relevant data from a rosbag and saves it in csv files.
"""
# imports
import roslib
import rosbag
from geometry_msgs.msg import Twist

bag = rosbag.Bag ('test.bag')

topic_status = '/fmData/status'
file_status= 'debug_roboteq_status.txt'
topic_deadman = '/fmSignals/deadman'
file_deadman = 'debug_deadman.txt'
topic_cmd_vel_left = '/fmSignals/cmd_vel_left'
file_cmd_vel_left = 'debug_cmd_vel_left.txt'
topic_cmd_vel_right = '/fmSignals/cmd_vel_right'
file_cmd_vel_right = 'debug_cmd_vel_right.txt'
topic_prop_status = '/fmInformation/propulsion_module_status'
file_prop_status = 'debug_prop_mod_status.txt'
topic_fb_left_status = '/fmInformation/propulsion_module_feedback_left'
file_fb_left_status = 'debug_feedback_left_status.txt'
topic_fb_right_status = '/fmInformation/propulsion_module_feedback_right'
file_fb_right_status = 'debug_feedback_right_status.txt'

def time_stamp (stamp):
	secs = stamp.secs
	msecs = int(stamp.nsecs/1000000.0+0.5)
	if msecs == 1000:
		secs += 1 
		msecs = 0		
	return '%d.%03d' % (secs, msecs)

# extract status
f = open (file_status, 'w')
for topic, msg, t in bag.read_messages(topics=[topic_status]):
	f.write ('%s,%s\n' % (time_stamp (t), msg.data))
f.close()

# extract deadman
f = open (file_deadman, 'w')
for topic, msg, t in bag.read_messages(topics=[topic_deadman]):
	f.write ('%s,%d\n' % (time_stamp (t), msg.data))
f.close()

# extract cmd_vel_left
f = open (file_cmd_vel_left, 'w')
for topic, msg, t in bag.read_messages(topics=[topic_cmd_vel_left]):
	f.write ('%s,%.3f\n' % (time_stamp (msg.header.stamp), msg.twist.linear.x))
f.close()

# extract cmd_vel_left
f = open (file_cmd_vel_right, 'w')
for topic, msg, t in bag.read_messages(topics=[topic_cmd_vel_left]):
	f.write ('%s,%.3f\n' % (time_stamp (msg.header.stamp), msg.twist.linear.x))
f.close()

# extract propulsion_mode_status
f = open (file_prop_status, 'w')
for topic, msg, t in bag.read_messages(topics=[topic_prop_status]):
	f.write ('%s,%.1f,%.1f,%.1f\n' % (time_stamp (t), msg.voltage, msg.current, msg.power))
f.close()

# extract propulsion_mode_status
f = open (file_fb_left_status, 'w')
for topic, msg, t in bag.read_messages(topics=[topic_fb_left_status]):
	f.write ('%s,%.1f,%.1f,%.1f\n' % (time_stamp (t), msg.velocity, msg.velocity_setpoint, msg.thrust))
f.close()

# extract propulsion_mode_status
f = open (file_fb_right_status, 'w')
for topic, msg, t in bag.read_messages(topics=[topic_fb_right_status]):
	f.write ('%s,%.1f,%.1f,%.1f\n' % (time_stamp (t), msg.velocity, msg.velocity_setpoint, msg.thrust))
f.close()

bag.close()


