#!/usr/bin/env python
#/****************************************************************************
# show_imu
# Copyright (c) 2013-2014, Kjeld Jensen <kjeld@frobomind.org>
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
Prints output from the IMU at a defined interval. 

2013-12-05 kjen, first version
2014-03-08 kjen, added calculation of yaw based on gyro and pitch, roll based on accelerometers. 
"""

import rospy
from math import atan2, asin, pi, sqrt
from sensor_msgs.msg import Imu

rad_to_deg = 180.0/pi
new_data = False
prev_msg_time = False

drift_publish_interval = 10.0 # seconds
drift_publish_time = 0.0
drift_begin_yaw = 0.0

gyro_yaw = 0.0
acc_pitch = 0.0
acc_roll = 0.0

orient_yaw = 0.0
orient_pitch = 0.0
orient_roll = 0.0

def on_imu_topic(msg):
	global new_data, gyro_yaw, acc_pitch, acc_roll, orient_yaw, orient_pitch, orient_roll, prev_msg_time, drift_begin_yaw, drift_publish_time
	new_data = True

	# determine yaw based on the z-axis gyro
	msg_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0
	if prev_msg_time != False:
		gyro_yaw += msg.angular_velocity.z * (msg_time - prev_msg_time)
	prev_msg_time = msg_time
	
	if drift_publish_time != 0.0:
		if msg_time >= drift_publish_time:
			print 'gyro yaw drift: %.9f [deg/s]' % ((gyro_yaw - drift_begin_yaw)/(msg_time - (drift_publish_time - drift_publish_interval))*rad_to_deg)
			drift_publish_time = msg_time + drift_publish_interval
			drift_begin_yaw = gyro_yaw			
	else:
		drift_publish_time = msg_time + drift_publish_interval
		drift_begin_yaw = gyro_yaw
	

	# determine pitch and roll based on the accelerometers
	ax = msg.linear_acceleration.x
	ay = msg.linear_acceleration.y
	az = msg.linear_acceleration.z
	g = sqrt(ax*ax + ay*ay + az*az)
	acc_pitch = asin(ay/-g)
	acc_roll = atan2(ax, az)

	# extract yaw, pitch and roll from the quaternion
	qx = msg.orientation.x
	qy = msg.orientation.y
	qz = msg.orientation.z
	qw = msg.orientation.w
	sqx = qx**2
	sqy = qy**2
	sqz = qz**2
	sqw = qw**2
	orient_yaw = atan2(2*(qx*qy + qw*qz), sqw + sqx - sqy - sqz)
	orient_pitch = asin((2*qy*qw) - (2*qx*qz))
	orient_roll = atan2((2*qy*qz) + (2*qx*qw), sqw + sqz - sqy - sqx)

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
	 	print 'gyro: yaw %4.0f    accelerometer: pitch %3.0f roll %3.0f    orientation: yaw %4.0f pitch %3.0f roll %3.0f' % (gyro_yaw*rad_to_deg, acc_pitch*rad_to_deg, acc_roll*rad_to_deg, orient_yaw*rad_to_deg, orient_pitch*rad_to_deg, orient_roll*rad_to_deg)

	# go back to sleep
	rospy.sleep(update_interval)

