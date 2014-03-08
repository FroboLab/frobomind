#!/usr/bin/env python
#/****************************************************************************
# Lidar 3d test script
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
Revision
2013-11-05 KJ First version
"""

import rospy
from msgs.msg import encoder
from math import pi, sin, cos

update_interval = 0.1 # [s]


rospy.init_node('print_roll_pitch')

encoder_topic = rospy.get_param("~encoder_topic", "/fmInformation/encoder_value")

def on_encoder_topic(msg):
	angle = msg.encoderticks*2*pi/16384.0
	roll = -pi/6 * sin(angle);
	pitch = -pi/6 * cos(angle);

	print 'ticks: %5d pitch: %5.1f roll: %5.1f' % (msg.encoderticks, pitch*180/pi, roll*180/pi)

rospy.Subscriber(encoder_topic, encoder, on_encoder_topic)


# loop until shutdown
while not rospy.is_shutdown():

	# sleep the defined interval
	rospy.sleep(update_interval)





