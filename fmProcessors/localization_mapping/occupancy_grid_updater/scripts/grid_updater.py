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
from std_msgs.msg import Float64

class Updater():
    """
        Converter 
    """
    def __init__(self):       
        # Init parameters
        self.sensor_width = rospy.get_param("~sensor_width",2.0)
        self.sensor_length = rospy.get_param("~sensor_length",1.0)
        self.sensor_offset_x = rospy.get_param("~sensor_offset_x",0.6)
        self.sensor_offset_y = rospy.get_param("~sensor_offset_y",0)
        self.sensor_outerrange = rospy.get_param("~sensor_outerrange",0.1)
        self.resolution = rospy.get_param("~resolution",0.05)
        self.period = rospy.get_param("~period",0.1)
        self.tf_offset_x = self.sensor_offset_x - self.sensor_outerrange
        self.tf_offset_y = (self.sensor_width/2) + self.sensor_outerrange
        self.trans_interval = 0.05 # 5cm
        self.angle_interval = 0.087 # 5 deg
        self.current_position_x = 0
        self.current_position_y = 0
        self.current_angle = 0
        self.last_position_x = self.current_position_x
        self.last_position_y = self.current_position_y
        self.last_angle = self.current_angle
        
        
        # Init topics and transforms
        self.map_pub = rospy.Publisher("/fmKnowledge/map", OccupancyGrid)
        self.sensor_sub = rospy.Subscriber("/wads", Float64, self.onSensor )
        self.sensor_sub = rospy.Subscriber("/odom", Odometry, self.onOdom )
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        
        # Init timers
        self.timer = rospy.Timer(rospy.Duration(self.period), self.publishTransform)
        self.timer = rospy.Timer(rospy.Duration(self.period), self.onTimer)
        
        self.sensor_value = 0
        self.map = OccupancyGrid()     
        self.map.header.frame_id = '/odom'
        self.map.info.map_load_time = rospy.Time.now() # The time at which the map was loaded
        self.map.info.resolution = self.resolution # The map resolution [m/cell]
        self.map.info.width = int( np.ceil(( self.sensor_width + (2*self.sensor_outerrange) ) / self.resolution )) # Map width [cells]
        self.map.info.height = int( np.ceil(( self.sensor_length + (2*self.sensor_outerrange) ) / self.resolution ))  # Map height [cells]
         
    def publishTransform(self,event):
        # Broadcast static transform from baselink to map
        self.br.sendTransform((self.tf_offset_x , self.tf_offset_y , 0),
                     tf.transformations.quaternion_from_euler(0, 0, - np.pi/2),
                     rospy.Time.now(),
                     "/map",
                     "/base_link")
        
    def onSensor(self,msg):
        # Save sensor value
        self.sensor_value = int( msg.data*20 )
        
    def onOdom(self,msg):
        self.current_position_x = msg.pose.pose.position.x
        self.current_position_y = msg.pose.pose.position.y
        quaternion = np.empty((4, ), dtype=np.float64)
        quaternion[0] = msg.pose.pose.orientation.x
        quaternion[1] = msg.pose.pose.orientation.y
        quaternion[2] = msg.pose.pose.orientation.z
        quaternion[3] = msg.pose.pose.orientation.w
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(quaternion)
        self.current_angle = yaw
    
    def onTimer(self,event):
        d_x = np.abs(self.current_position_x - self.last_position_x)
        d_y = np.abs(self.current_position_y - self.last_position_y)
        d_distance = np.sqrt(d_x*d_x + d_y*d_y)        
        d_angle = np.abs(self.current_angle - self.last_angle)
        if d_distance > self.trans_interval or d_angle > self.angle_interval:
            self.last_position_x = self.current_position_x
            self.last_position_y = self.current_position_y
            self.last_angle = self.current_angle
            self.publishMap()
                
    def publishMap(self):
        # Time stamp the map
        self.map.header.stamp = rospy.Time.now() 
        
        # Place local map on global map
        try :      
            (trans,rot) = self.listener.lookupTransform('/odom', '/map', rospy.Time(0))
            self.map.info.origin.position.x = trans[0] 
            self.map.info.origin.position.y = trans[1]
            self.map.info.origin.orientation.x = rot[0]
            self.map.info.origin.orientation.y = rot[1]
            self.map.info.origin.orientation.z = rot[2]
            self.map.info.origin.orientation.w = rot[3]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        
        # Generate map from sensor model
        self.map.data = [self.sensor_value] * (self.map.info.width * self.map.info.height)
#        for x in range(self.map.info.width) :
#            self.map.data[x] = self.sensor_value
#            self.map.data[self.map.info.width+x] = self.sensor_value
#            self.map.data[self.map.info.width*(self.map.info.height-1)+x] = self.sensor_value
#            self.map.data[self.map.info.width*(self.map.info.height-2)+x] = self.sensor_value
#        for y in range(self.map.info.height) :
#            self.map.data[self.map.info.width*y] = self.sensor_value
#            self.map.data[self.map.info.width*y+1] = self.sensor_value
#            self.map.data[self.map.info.width*y+self.map.info.width-1] = self.sensor_value
#            self.map.data[self.map.info.width*y+self.map.info.width-2] = self.sensor_value
        
        # Publish local map
        self.map_pub.publish(self.map)


if __name__ == '__main__':
    rospy.init_node('cmd_vel_converter')
    node = Updater()
    rospy.spin()
    



    