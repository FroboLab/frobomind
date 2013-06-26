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
from tf.transformations import euler_from_quaternion
#from geometry_msgs.msg import Quaternion
from math import pi

bag = rosbag.Bag ('test.bag')
topic_pose = '/fmKnowledge/pose'
topic_wptnav_status = '/fmData/wptnav_status'
topic_cmd_vel = '/fmSignals/cmd_vel'
topic_cmd_vel_left = '/fmSignals/cmd_vel_left'
topic_cmd_vel_right = '/fmSignals/cmd_vel_right'
topic_velocity_left = '/fmData/velocity_left'
topic_velocity_right = '/fmData/velocity_right'
topic_power_left = '/fmData/power_left'
topic_power_right = '/fmData/power_right'

def time_stamp (stamp):
	secs = stamp.secs
	msecs = int(stamp.nsecs/1000000.0+0.5)
	if msecs == 1000:
		secs += 1 
		msecs = 0		
	return '%d.%03d' % (secs, msecs)

rad_to_deg = 180/pi

# extract rosout
f = open ('sim_rosout.txt', 'w')
for topic, msg, t in bag.read_messages(topics=['/rosout']):
	f.write ('%s,%d,%s,%s\n' % (time_stamp(msg.header.stamp), msg.level, msg.name, msg.msg))
f.close()

# extract pose data
f = open ('sim_pose.txt', 'w')
for topic, msg, t in bag.read_messages(topics=[topic_pose]):
	(roll,pitch,yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, \
		msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
	f.write ('%s,%.3f,%.3f,%.3f,%.3f,%.3f\n' % (time_stamp(msg.header.stamp), msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, msg.twist.twist.linear.x,  msg.twist.twist.angular.z))
f.close()

# extract wptnav_status data
f = open ('sim_wptnav_status.txt', 'w')
for topic, msg, t in bag.read_messages(topics=[topic_wptnav_status]):
	f.write ('%s,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.3f,%.3f\n' % (time_stamp(msg.header.stamp), \
		msg.mode, msg.b_easting, msg.b_northing, msg.a_easting, msg.a_northing, \
		msg.easting, msg.northing, msg.distance_to_b, msg.bearing_to_b*rad_to_deg, msg.heading_err*rad_to_deg, msg.distance_to_ab_line, \
		msg.target_easting, msg.target_northing, msg.target_distance, msg.target_bearing*rad_to_deg, msg.target_heading_err*rad_to_deg, \
		msg.linear_speed, msg.angular_speed))
f.close()

# extract cmd_vel data
f = open ('sim_cmd_vel.txt', 'w')
for topic, msg, t in bag.read_messages(topics=[topic_cmd_vel]):
	f.write ('%s,%.6f,%.6f\n' % (time_stamp(msg.header.stamp), msg.twist.linear.x, msg.twist.angular.z))
f.close()

# extract cmd_vel_left data
f = open ('sim_cmd_vel_left.txt', 'w')
for topic, msg, t in bag.read_messages(topics=[topic_cmd_vel_left]):
	f.write ('%s,%.6f\n' % (time_stamp(msg.header.stamp), msg.twist.linear.x))
f.close()

# extract cmd_vel_right data
f = open ('sim_cmd_vel_right.txt', 'w')
for topic, msg, t in bag.read_messages(topics=[topic_cmd_vel_right]):
	f.write ('%s,%.6f\n' % (time_stamp(msg.header.stamp), msg.twist.linear.x))
f.close()

# extract velocity_left data
f = open ('sim_velocity_left.txt', 'w')
for topic, msg, t in bag.read_messages(topics=[topic_velocity_left]):
	f.write ('%s,%.6f\n' % (time_stamp(t), msg.data))
f.close()

# extract velocity_right data
f = open ('sim_velocity_right.txt', 'w')
for topic, msg, t in bag.read_messages(topics=[topic_velocity_right]):
	f.write ('%s,%.6f\n' % (time_stamp(t), msg.data))
f.close()

# extract power_left data
f = open ('sim_power_left.txt', 'w')
for topic, msg, t in bag.read_messages(topics=[topic_power_left]):
	f.write ('%s,%.6f\n' % (time_stamp(msg.header.stamp), msg.data))
f.close()

# extract power_right data
f = open ('sim_power_right.txt', 'w')
for topic, msg, t in bag.read_messages(topics=[topic_power_right]):
	f.write ('%s,%.6f\n' % (time_stamp(msg.header.stamp), msg.data))
f.close()

bag.close()


