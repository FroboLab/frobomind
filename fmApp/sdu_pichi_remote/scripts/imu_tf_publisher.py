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
import rospy,math,tf,csv,sys
import numpy as np
from simple_2d_math.plot import Vectorplot
from simple_2d_math.vector import Vector
from pylab import ion
import matplotlib.pyplot as pyplot
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion,Point

class IMUTransformPublisher():
    def __init__(self):        
        # Init transform
        self.br = tf.TransformBroadcaster()
        self.quaternion = np.empty((4, ), dtype=np.float64)
        self.odom_msg = Odometry()
        self.tf = tf.TransformListener()
        
        # Init subscriber
        self.imu_sub = rospy.Subscriber('/fmInformation/imu', Imu, self.onImu )
        self.odom_pub = rospy.Publisher('/fmKnowledge/odom_combined', Odometry)


    def multiply(self,q1,q2):
        result = np.empty((4, ), dtype=np.float64)
        result[0] = q1[3] * q2[0] + q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1]
        result[1] = q1[3] * q2[1] + q1[1] * q2[3] + q1[2] * q2[0] - q1[0] * q2[2]
        result[2] = q1[3] * q2[2] + q1[2] * q2[3] + q1[0] * q2[1] - q1[1] * q2[0]
        result[3] = q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2]
        return result
    
    def quat(self,x,y,z,w):
        new = np.empty((4, ), dtype=np.float64)
        new[0] = x
        new[1] = y
        new[2] = z
        new[3] = w
        return new
                
    
    def onImu(self,msg):       
        # Z-axis is flipped due to ENU        
        self.quaternion[0] = msg.orientation.x
        self.quaternion[1] = msg.orientation.y
        self.quaternion[2] = msg.orientation.z
        self.quaternion[3] = msg.orientation.w
        
        rot = self.quat(0,1,0,math.pi/3)
        self.quaternion = self.multiply(self.quaternion, rot)
        rot = self.quat(1,0,0,math.pi/3)
        self.quaternion = self.multiply(self.quaternion, rot)
        rot = self.quat(0,0,1,(2*math.pi)/3)
        self.quaternion = self.multiply(self.quaternion, rot)

        self.br.sendTransform((0,0,0),
                     self.quaternion,
                     rospy.Time.now(),
                     "map",
                     "mast_top")
        
        try:
            (position,orientation) = self.tf.lookupTransform("world_frame","mast_bottom", rospy.Time(0)) 
            self.odom_msg.header.stamp = rospy.Time.now()
            self.odom_msg.header.frame_id = 'world_frame'
            self.odom_msg.child_frame_id = 'mast_bottom'
            self.odom_msg.pose.pose.position = Point(position[0],position[1],0)
            self.odom_msg.pose.pose.orientation = Quaternion(orientation[0],orientation[1],orientation[2],orientation[3])
            self.odom_pub.publish(self.odom_msg)
        except (tf.LookupException, tf.ConnectivityException, tf.Exception),err:
            rospy.loginfo("Transform error: %s",err)
            
                
if __name__ == '__main__':
    rospy.init_node('imu_transform_pub')
    imu = IMUTransformPublisher()
    rospy.spin()

