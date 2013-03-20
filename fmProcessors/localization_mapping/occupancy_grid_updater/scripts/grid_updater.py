#!/usr/bin/env python
#/****************************************************************************
# FroboMind cmd_vel_converter.py
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
import rospy, tf
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from msgs.msg import FloatStamped

class Updater():
    """
        Converter 
    """
    def __init__(self):
        self.map_pub = rospy.Publisher("/fmKnowledge/map", OccupancyGrid)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.onOdometry)
        self.sensor_sub = rospy.Subscriber("/fmInformation/wads", FloatStamped, self.onSensor )
        
        self.br = tf.TransformBroadcaster()
        
        self.timer = rospy.Timer(rospy.Duration(0.2), self.publishMap)
        self.timer = rospy.Timer(rospy.Duration(0.2), self.publishTransform)
        
        self.sensor_value = 0
        self.map = OccupancyGrid()     
        self.map.info.map_load_time = rospy.Time.now() # The time at which the map was loaded
        self.map.info.resolution = 0.1 # The map resolution [m/cell]
        self.map.info.width = 30 # Map width [cells]
        self.map.info.height = 30 # Map height [cells]
        # The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.
        self.map.info.origin.position.x = 0
        self.map.info.origin.position.y = 0
#        self.map.info.origin.orientation = tf.transformations.quaternion_from_euler(0, 0, 0)
        # The map data, in row-major order, starting with (0,0).  Occupancy probabilities are in the range [0,100].  Unknown is -1.
        self.map.data = [0] * (self.map.info.width * self.map.info.height)
 
    def onOdometry(self,msg):
        self.map.info.origin.position.x = msg.pose.pose.position.x 
        self.map.info.origin.position.y = msg.pose.pose.position.y
        self.map.info.origin.orientation = msg.pose.pose.orientation
        
    def publishTransform(self,event):
        self.br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 1.5),
                     rospy.Time.now(),
                     "/map",
                     "/odom")
        
    def onSensor(self,msg):
        self.sensor_value = msg.data
        
    def publishMap(self,event):
        self.map.header.stamp = rospy.Time.now()
        for i in range(10) :
            for j in range(10) :
                self.map.data[((i+10)*30)+10+j] = 100
        self.map_pub.publish(self.map)


if __name__ == '__main__':
    rospy.init_node('cmd_vel_converter')
    node = Updater()
    rospy.spin()
    



    