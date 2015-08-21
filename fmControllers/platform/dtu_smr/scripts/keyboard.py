#!/usr/bin/env python
#/****************************************************************************
# DTU SMR example
# Copyright (c) 2013, Morten Kjaergaard
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

import roslib; roslib.load_manifest('smr')
import rospy
import wx
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class MyForm(wx.Frame):

    
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist)
        rospy.init_node('keyboard')
        
        wx.Frame.__init__(self, None, wx.ID_ANY, "Key Press Tutorial")
 
        # Add a panel so it looks the correct on all platforms
        panel = wx.Panel(self, wx.ID_ANY)
        btn = wx.Button(panel, label="OK")
 
        btn.Bind(wx.EVT_KEY_DOWN, self.onKeyPress)
 
    def onKeyPress(self, event):
        keycode = event.GetKeyCode()
        print keycode
        if keycode == wx.WXK_UP:
            print "you pressed up!"
            self.pub.publish(Twist(Vector3(0.2,0,0),Vector3(0,0,0)))
        elif keycode == wx.WXK_DOWN:
            print "you pressed down!"
            self.pub.publish(Twist(Vector3(-0.2,0,0),Vector3(0,0,0)))
        elif keycode == wx.WXK_LEFT:
            print "you pressed left!"
            self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0.3)))
        elif keycode == wx.WXK_RIGHT:
            print "you pressed right!"
            self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,-0.3)))
        event.Skip()
 
# Run the program
if __name__ == "__main__":
    app = wx.PySimpleApp()
    frame = MyForm()
    frame.Show()
    app.MainLoop()
