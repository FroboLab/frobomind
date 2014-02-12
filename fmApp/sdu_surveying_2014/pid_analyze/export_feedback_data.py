#!/usr/bin/env python
#*****************************************************************************
# export_data
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

bag = rosbag.Bag ('test.bag')
topic_fb = '/fmData/wheel_right_nmea_in'

def time_stamp (stamp):
	secs = stamp.secs
	msecs = int(stamp.nsecs/1000000.0+0.5)
	if msecs == 1000:
		secs += 1 
		msecs = 0		
	return '%d.%03d' % (secs, msecs)



# extract wheel feedback data
f = open ('wheel_feedback.txt', 'w')
for topic, msg, t in bag.read_messages(topics=[topic_fb]):
	f.write ('%s,%s,%s,%s,%s,%ld,%ld,%s,%s,%s,%s\n' % (time_stamp(msg.header.stamp), \
		msg.data[0], msg.data[1], msg.data[2], msg.data[3], int(msg.data[4])/50, \
		int(msg.data[5])/50, msg.data[6], msg.data[7], msg.data[8], msg.data[9]))
f.close()

bag.close()


