#!/usr/bin/env python
#*****************************************************************************
# KP2000 projection conversion test
# Copyright (c) 2013, Leon Bonde Larsen <leon@frobomind.org>
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

"""
import rospy,math,tf,csv,sys
import numpy as np
from simple_2d_math.plot import Vectorplot
from simple_2d_math.vector import Vector
from pylab import ion
import matplotlib.pyplot as pyplot
from sensor_msgs.msg import Imu

class GPStest():
    def __init__(self,file):
        # Task parameters
        self.running = True
        
        # Class variables
        self.origin = Vector(0,0)
        self.current = Vector(0,0)
        self.position_list = []
        
        # init plot
        self.fig = pyplot.figure(num=None, figsize=(8, 8), dpi=80, facecolor='w', edgecolor='k')
        self.area = 2
        ion()
        
        # Init transform
        self.tf = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.quaternion = np.empty((4, ), dtype=np.float64)
        
        # Init node
        self.rate = rospy.Rate(10)
        
        # Init subscriber
        self.imu_sub = rospy.Subscriber('/fmInformation/imu', Imu, self.onImu )
        
        # Init csv
        self.file = file
            
    def initPlot(self):
        pyplot.axis('equal')
        pyplot.axis([self.origin[0]-self.area/2,self.origin[0]+self.area/2,self.origin[1]-self.area/2,self.origin[1]+self.area/2])
        pyplot.grid()
        
        
    def spin(self):
        rospy.sleep(5)
        self.update()
        self.origin = self.current
        while self.running:
            
            self.update()
            self.draw()
            
            if rospy.is_shutdown() :
                self.running = False
            
            # Block   
            try :
                self.rate.sleep()
            except rospy.ROSInterruptException:
                self.running = False
        
        self.saveList()

    def update(self):  
        try:
            now = rospy.Time.now() - rospy.Duration(0.5)
            (position,orientation) = self.tf.lookupTransform("base_link", "rod_base", rospy.Time(0))  
            self.current = Vector(position[0],position[1])        
        except (tf.LookupException, tf.ConnectivityException, tf.Exception),err:
            rospy.loginfo("Transform error: %s",err)     
        
        
    def draw(self):
        pyplot.plot(self.current[0],self.current[1], 'ro')
        pyplot.draw()  
        self.position_list.append( [self.current[0] , self.current[1]] )
        
    def onImu(self,msg):
        self.quaternion[0] = msg.orientation.x
        self.quaternion[1] = msg.orientation.y
        self.quaternion[2] = msg.orientation.w
        self.quaternion[3] = msg.orientation.z
        self.br.sendTransform((0, 0, 0),
                     self.quaternion,
                     rospy.Time.now(),
                     "odom_combined",
                     "imu_link")
        
    def saveList(self):
        with open(self.file, 'wb') as csvfile:
            writer = csv.writer(csvfile, delimiter=',', quotechar=' ' , quoting=csv.QUOTE_MINIMAL)
            writer.writerow([self.origin[0],self.origin[1]])
            writer.writerows(self.position_list)
            print("Data saved to " + self.file)
    
if __name__ == '__main__':
    rospy.init_node('gps_test')
    test = GPStest( str(sys.argv[1]) )
    test.spin()

