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
from msgs.msg import IntStamped

class Plotter():
    """
        Converter 
    """
    def __init__(self):
        # Init node 
        self.ticks_pr_meter = 1285.0
        self.right_belt_converion_factor = 1285.0/682.0
        self.ticks_to_mps_left = 1.0/self.ticks_pr_meter
        self.ticks_to_mps_right = self.right_belt_converion_factor*self.ticks_pr_meter
        self.left = []
        self.right = []
        self.last_left = 0.0
        self.last_right = 0.0
        self.left_value = 0.0
        self.right_value = 0.0
        self.filter_left = [0.0]*10
        self.filter_right = [0.0]*10
        self.ptr_left = 0
        self.ptr_right = 0
        self.time_left = rospy.Time.now()
        self.time_right = rospy.Time.now()
        self.initialised_left = False
        self.initialised_right = False
        self.r = rospy.Rate(50)    
        self.left_sub = rospy.Subscriber("/fmInformation/encoder_left", IntStamped, self.onLeft )
        self.right_sub = rospy.Subscriber("/fmInformation/encoder_right", IntStamped, self.onRight )
        
        ion() # turn interaction mode on
        
        plt.figure(num=None, figsize=(12,6), dpi=80, facecolor='w', edgecolor='w')
        title ('Velocity')

        ylabel('[m/s]')
        grid (True)  
         
    def onLeft(self, msg):
        now = rospy.Time.now()
        period = (now - self.time_left).to_sec()
        self.time_left = now
        if self.initialised_left :
            self.left_value = self.last_left - msg.data
            self.filter_left[self.ptr_left] = self.left_value
            self.ptr_left += 1
            if self.ptr_left == len(self.filter_left) :
                self.ptr_left = 0
            self.left.append(self.ticks_to_mps( (sum(self.filter_left)/len(self.filter_left)) , period))
            self.last_left = msg.data
        else :
            self.last_left = msg.data
            self.initialised_left = True
        
    def onRight(self, msg):
        now = rospy.Time.now()
        period = (now - self.time_right).to_sec() 
        self.time_right = now
        if self.initialised_right :
             # Make relative
            self.right_value = -(self.last_right - msg.data)
            
            #Push to buffer
            self.filter_right[self.ptr_right] = self.right_value*self.right_belt_converion_factor
            self.ptr_right += 1
            if self.ptr_right == len(self.filter_right) :
                self.ptr_right = 0
            
            # Append average of buffer to plot
            self.right.append(self.ticks_to_mps( (sum(self.filter_right)/len(self.filter_right)) , period)) 
            
            #upkeep
            self.last_right = msg.data
            
        else : # if not initialised
            self.last_right = msg.data
            self.initialised_right = True
        
    
    def ticks_to_mps(self, ticks, period) :
        return (ticks/self.ticks_pr_meter)/period
    
    # update loop
    def spin(self):
        while not rospy.is_shutdown():
            plt.figure(1)
            plt.plot(self.left,'r')
            plt.plot(self.right,'b')
            plt.draw() 
            self.r.sleep()       

if __name__ == '__main__':
    rospy.init_node('state_printer')
    node = Plotter()
    node.spin()
    



    