#!/usr/bin/env python
#*****************************************************************************
# Export real time analyze data
# Copyright (c) 2014, Kjeld Jensen <kjeld@frobomind.org>
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
import sys
import roslib
import rosbag

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


topic_rt_timing = '/fmInformation/rt_timing'
topic_cpu_load = '/fmInformation/cpu_load'
topic_memory_load = '/fmInformation/memory_load'

def time_stamp (stamp):
	secs = stamp.secs
	msecs = int(stamp.nsecs/1000000.0+0.5)
	if msecs == 1000:
		secs += 1 
		msecs = 0		
	return '%d.%03d' % (secs, msecs)

# extract data
f = open ('rosbag_rt_timing.txt', 'w')
for topic, msg, t in bag.read_messages(topics=[topic_rt_timing]):
	f.write ('%s,%d\n' % (time_stamp (msg.header.stamp), msg.data))
f.close()

f = open ('rosbag_cpu_load.txt', 'w')
for topic, msg, t in bag.read_messages(topics=[topic_cpu_load]):
	f.write ('%s,%.3f\n' % (time_stamp (msg.header.stamp), msg.data))
f.close()

f = open ('rosbag_memory_load.txt', 'w')
for topic, msg, t in bag.read_messages(topics=[topic_memory_load]):
	f.write ('%s,%.3f\n' % (time_stamp (msg.header.stamp), msg.data))
f.close()

bag.close()


