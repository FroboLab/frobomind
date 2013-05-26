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
        # Init node
        self.encoder_l_pub = rospy.Publisher("/fmInformation/enc_left", encoder)
        self.encoder_r_pub = rospy.Publisher("/fmInformation/enc_right", encoder)
        self.encoder_l_sub = rospy.Subscriber("/fmInformation/encoder_left", IntStamped, self.onIntStampedLeft )
        self.encoder_r_sub = rospy.Subscriber("/fmInformation/encoder_right", IntStamped, self.onIntStampedRight )
        self.encoder_l = encoder() 
        self.encoder_r = encoder()
        self.last_left = 0.0
        self.last_right = 0.0
        self.initialised_l = False 
        self.initialised_r = False    
 
    def onIntStampedLeft(self,msg):
        if self.initialised_l :   
            self.encoder_l.encoderticks = msg.data - self.last_left
            self.encoder_l.header.stamp = msg.header.stamp       
            self.encoder_l_pub.publish(self.encoder_l)
            self.last_left = msg.data
        else:
            self.last_left = msg.data
            self.initialised_l = True
        
    def onIntStampedRight(self,msg):
        if self.initialised_r :
            self.encoder_r.encoderticks = msg.data - self.last_right
            self.encoder_r.header.stamp = msg.header.stamp       
            self.encoder_r_pub.publish(self.encoder_r)
            self.last_right = msg.data
        else:
            self.last_right = msg.data
            self.initialised_r = True

if __name__ == '__main__':
    rospy.init_node('cmd_vel_converter')
    node = Converter()
    rospy.spin()
    



    