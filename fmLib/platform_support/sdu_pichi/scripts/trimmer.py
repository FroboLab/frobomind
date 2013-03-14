#!/usr/bin/env python
#/****************************************************************************
# FroboMind init_serial_nmea.py
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
"""@package trimmer
    GUI for trimming PID parameters.

    Publishes set commands on /fmData/tx.
"""
import rospy,sys
from pid_trimmer import pid_trimmer
from msgs.msg import serial
from PyQt4 import QtCore, QtGui

class sender():
    def __init__(self):
        """Constructor. Instantiates the Qt GUI and inits the ROS node"""
        # Init GUI and register callback
        self.app = QtGui.QApplication(sys.argv)
        self.Form = QtGui.QWidget()
        self.ui = pid_trimmer.Ui_Form()
        self.ui.setupUi(self.Form)
        self.ui.register_cb(self.onUpdate)
        
        # Init node an set up publisher  
        rospy.init_node('PID_trimmer', anonymous=True)      
        self.serial_pub = rospy.Publisher("/fmData/tx", serial)
        self.serial_msg = serial()
        self.rate = rospy.Rate(10)
    
    def spin(self):
        """Method to start GUI"""
        self.Form.show()
        sys.exit(self.app.exec_())
        
    def onUpdate(self,p_gain,i_gain,d_gain,ramp_up,ramp_down):
        """Callback method to publish set commands"""
        self.serial_msg.header.stamp = rospy.Time.now()
        if p_gain :
            self.serial_msg.data = "^KP 1 " + p_gain
            self.serial_pub.publish(self.serial_msg)
            self.serial_msg.data = "^KP 2 " + p_gain
            self.serial_pub.publish(self.serial_msg)
            self.rate.sleep()
        if i_gain :
            self.serial_msg.data = "^KI 1 " + i_gain
            self.serial_pub.publish(self.serial_msg)
            self.serial_msg.data = "^KI 2 " + i_gain
            self.serial_pub.publish(self.serial_msg)
            self.rate.sleep()
        if d_gain :
            self.serial_msg.data = "^KD 1 " + d_gain
            self.serial_pub.publish(self.serial_msg)
            self.serial_msg.data = "^KD 2 " + d_gain
            self.serial_pub.publish(self.serial_msg)
            self.rate.sleep()
        if ramp_up :
            self.serial_msg.data = "^MAC 1 " + ramp_up
            self.serial_pub.publish(self.serial_msg)
            self.serial_msg.data = "^MAC 2 " + ramp_up
            self.serial_pub.publish(self.serial_msg)
            self.rate.sleep()
        if ramp_down :
            self.serial_msg.data = "^MDEC 1 " + ramp_down
            self.serial_pub.publish(self.serial_msg)
            self.serial_msg.data = "^MDEC 2 " + ramp_down
            self.serial_pub.publish(self.serial_msg)
            self.rate.sleep()

if __name__ == "__main__":
    """main function to instantiate the class and spin"""
    node = sender()
    node.spin()
