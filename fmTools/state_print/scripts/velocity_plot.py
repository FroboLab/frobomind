#!/usr/bin/env python
#/****************************************************************************
# FroboMind 
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
import rospy, tf, math, numpy
import matplotlib.pyplot as plt
from pylab import ion, plot, axis, grid, title, xlabel, ylabel, draw
from msgs.msg import PropulsionModuleFeedback

class Plotter():
    """
        Converter 
    """
    def __init__(self):
        # Init node 
        self.left_error = []
        self.right_error = []
        self.left_velocity = []
        self.right_velocity = []
        self.left_setpoint = []
        self.right_setpoint = []

        self.r = rospy.Rate(50)    
        self.left_sub = rospy.Subscriber("/fmInformation/propulsion_module_feedback_left", PropulsionModuleFeedback, self.onLeft )
        self.right_sub = rospy.Subscriber("/fmInformation/propulsion_module_feedback_right", PropulsionModuleFeedback, self.onRight )
        
        ion() # turn interaction mode on
        
        fig1 = plt.figure(num=1, figsize=(12,6), dpi=80, facecolor='w', edgecolor='w')
        grid (True) 
        title ('Left')
        ylabel('t [s]')
        ylabel('velocity [m/s]')
        fig2 = plt.figure(num=2, figsize=(12,6), dpi=80, facecolor='w', edgecolor='w')
        title ('Right')
        ylabel('t [s]')
        ylabel('velocity [m/s]')
        grid (True)  
         
    def onLeft(self, msg):
        self.left_velocity.append(msg.velocity)
        self.left_setpoint.append(msg.velocity_setpoint)
        self.left_error.append(msg.velocity - msg.velocity_setpoint)

        
    def onRight(self, msg):
        self.right_velocity.append(msg.velocity)
        self.right_setpoint.append(msg.velocity_setpoint)
        self.right_error.append(msg.velocity - msg.velocity_setpoint)
    
    # update loop
    def spin(self):
        while not rospy.is_shutdown():
            plt.figure(1)
            plt.plot(self.left_error,'r', label="error")
            plt.plot(self.left_velocity,'b', label="measured")
            plt.plot(self.left_setpoint,'g', label="setpoint")
            plt.figure(2)
            plt.plot(self.right_error,'r', label="error")
            plt.plot(self.right_velocity,'b', label="error")
            plt.plot(self.right_setpoint,'g', label="error")
#            legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,ncol=2, mode="expand", borderaxespad=0.)
            plt.draw() 
            self.r.sleep()       

if __name__ == '__main__':
    rospy.init_node('state_printer')
    node = Plotter()
    node.spin()
    



    