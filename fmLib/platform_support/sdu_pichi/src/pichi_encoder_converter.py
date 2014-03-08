#!/usr/bin/env python
#/****************************************************************************
# FroboMind cmd_vel_converter.py
# Copyright (c) 2011-2013, author Leon Bonde Larsen <leon@bondelarsen.dk>
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
import rospy
from msgs.msg import IntStamped, encoder

class Converter():
    """
        Converter 
    """
    def __init__(self):
		rospy.loginfo(rospy.get_name() + ": Start")

 		# get topic names
		self.left_sub_topic = rospy.get_param("~left_sub",'/fmInformation/encoder_left')
		self.right_sub_topic = rospy.get_param("~right_sub",'/fmInformation/encoder_right')
		self.left_pub_topic = rospy.get_param("~left_pub",'/fmInformation/enc_left')
		self.right_pub_topic = rospy.get_param("~right_pub",'/fmInformation/enc_right')

		# initialize message publishers
		self.encoder_l_pub = rospy.Publisher(self.left_pub_topic, encoder)
		self.encoder_r_pub = rospy.Publisher(self.right_pub_topic, encoder)
		self.encoder_l = encoder() 
		self.encoder_r = encoder()     
 
		# initialize subscribers
		self.encoder_l_sub = rospy.Subscriber(self.left_sub_topic, IntStamped, self.onIntStampedLeft )
		self.encoder_r_sub = rospy.Subscriber(self.right_sub_topic, IntStamped, self.onIntStampedRight )

    def onIntStampedLeft(self,msg):
        self.encoder_l.encoderticks = msg.data * -1.0
        self.encoder_l.header.stamp = msg.header.stamp       
        self.encoder_l_pub.publish(self.encoder_l)
        
    def onIntStampedRight(self,msg):
        self.encoder_r.encoderticks = msg.data * 2.0
        self.encoder_r.header.stamp = msg.header.stamp       
        self.encoder_r_pub.publish(self.encoder_r)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_converter')
    node = Converter()
    rospy.spin()
   

    
