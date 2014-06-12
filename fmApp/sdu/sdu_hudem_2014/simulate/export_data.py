#!/usr/bin/env python
#*****************************************************************************
# export_simulation_data
# Copyright (c) 2013-2014, Kjeld Jensen <kjeld@frobomind.org>
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
#*****************************************************************************
"""
This script extract relevant data from a rosbag and saves it in csv files.
"""
# imports
import sys
import roslib
import rosbag
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion

arg1 =  sys.argv[1:][0]

if arg1 != '':
	bag = rosbag.Bag (arg1)
else:
	bag = rosbag.Bag ('test.bag')
	
def time_stamp (stamp):
	secs = stamp.secs
	msecs = int(stamp.nsecs/1000000.0+0.5)
	if msecs == 1000:
		secs += 1 
		msecs = 0		
	return '%d.%03d' % (secs, msecs)


topic_reference_gnss = '/fmInformation/gpgga_tranmerc_ref'

# extract pose data
f = open ('sim_pose.txt', 'w')
for topic, msg, t in bag.read_messages(topics=['/fmKnowledge/pose']):
	(roll,pitch,yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, \
		msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
	f.write ('%s,%.3f,%.3f,%.6f\n' % (time_stamp(msg.header.stamp), msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))
f.close()

# extract GPGGA data
#f = open ('sim_gnss.txt', 'w')
#for topic, msg, t in bag.read_messages(topics=['/fmInformation/gpgga_tranmerc']):
#	f.write ('%s,%s,%.10f,%.10f,%.4f,%.4f,%d,%d,%.2f\n' % (time_stamp(msg.header.stamp), msg.time, msg.lat, msg.lon, msg.easting, #msg.northing, msg.fix, msg.sat, msg.hdop))

# extract GPGGA data
#f = open ('sim_ref_gnss.txt', 'w')
#for topic, msg, t in bag.read_messages(topics=[topic_reference_gnss]):
#	f.write ('%s,%s,%.10f,%.10f,%.4f,%.4f,%d,%d,%.2f\n' % (time_stamp(msg.header.stamp), msg.time, msg.lat, msg.lon,  msg.easting, #msg.northing, msg.fix, msg.sat, msg.hdop))

# extract WADS data
wads_found = False
f = open ('sim_wads.txt', 'w')
for topic, msg, t in bag.read_messages(topics=['/fmData/nmea_from_wads']):
	wads_found = True
	f.write ('%s,%s\n' % (time_stamp(msg.header.stamp), msg.data[0]))
f.close()
if wads_found == False:
	f = open ('sim_wads.txt', 'w')
	for topic, msg, t in bag.read_messages(topics=['/fmInformation/wads']):
		f.write ('%s,%.0f\n' % (time_stamp(msg.header.stamp), msg.data))
	f.close()


bag.close()


