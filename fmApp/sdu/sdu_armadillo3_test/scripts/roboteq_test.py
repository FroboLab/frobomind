#!/usr/bin/env python
#/****************************************************************************
# FroboMind init_serial_nmea.py
# Copyright (c) 2011-2013, Leon Bonde Larsen <leon@bondelarsen.dk>
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
    This script is for testing the Armadillo system without a roboteq controller present.
"""
import rospy
import math
from msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *

class TestInterface():
    def __init__(self):
        # Init node and setup topics
        rospy.init_node('test_interface')
        self.publisher_left = rospy.Publisher("/fmData/left_belt_rx", serial)
        self.tx_sub_left = rospy.Subscriber("/fmData/left_belt_tx", serial, self.onTxLeft)
        self.rx_sub_left = rospy.Subscriber("/fmData/left_belt_rx", serial, self.onRxLeft)
        self.publisher_right = rospy.Publisher("/fmData/right_belt_rx", serial)
        self.tx_sub_right = rospy.Subscriber("/fmData/right_belt_tx", serial, self.onTxRight)
        self.rx_sub_right = rospy.Subscriber("/fmData/right_belt_rx", serial, self.onRxRight)
        self.msg = serial()

        # Spin
        try:
            while not rospy.is_shutdown():
                rospy.sleep(10)
        except rospy.ROSInterruptException:
            pass
    
    def onRxLeft(self,msg):
#        print(msg.data)
        return 0
            
    def onTxLeft(self,msg):
#        print(msg.data)
        if "?FID" in msg.data :
            self.msg.header.stamp = rospy.Time.now()
            self.msg.data = "FID=Roboteq blah blah"
            self.publisher_left.publish(self.msg)
        else :
            self.msg.header.stamp = rospy.Time.now()
            self.msg.data = "CB=100"
            self.publisher_left.publish(self.msg)
            
    def onRxRight(self,msg):
#        print(msg.data)
        return 0
            
    def onTxRight(self,msg):
#        print(msg.data)
        if "?FID" in msg.data :
            self.msg.header.stamp = rospy.Time.now()
            self.msg.data = "FID=Roboteq blah blah"
            self.publisher_right.publish(self.msg)
        else :
            self.msg.header.stamp = rospy.Time.now()
            self.msg.data = "CB=100"
            self.publisher_right.publish(self.msg)

if __name__ == '__main__':
    node = TestInterface()
    rospy.spin()
    



    