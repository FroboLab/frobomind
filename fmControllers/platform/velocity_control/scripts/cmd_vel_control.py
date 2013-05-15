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
import rospy, tf, math
from simple_2d_math.vector import Vector
from geometry_msgs.msg import TwistStamped
from velocity.control import Controller
from tf import TransformListener
from dynamic_reconfigure.server import Server
from velocity import ParametersConfig

class CmdVelController():
    """
        Converter for using FroboMind with stage. 
        Takes TwistStamped message from /fmSignals/cmd_vel and parses as Twist message on /cmd_vel
    """
    def __init__(self):
        # Get parameters
        self.cmd_vel_sub = rospy.get_param("~cmd_vel_sub","/fmSignals/cmd_vel_sp")
        self.cmd_vel_pub = rospy.get_param("~cmd_vel_sub","/fmSignals/cmd_vel")
        self.frame_id = rospy.get_param("~frame_id",'base_link')
        self.world_frame = rospy.get_param("~world_frame",'map')
        self.rate = rospy.Rate(50.0)
        
        # Init node
        self.twist_sub = rospy.Subscriber(self.cmd_vel_sub, TwistStamped, self.onTwist )
        self.twist_pub = rospy.Publisher(self.cmd_vel_pub, TwistStamped)
        
        self.twist = Twist()
        self.tf = TransformListener()
        self.controller = Controller()
 
    def onTwist(self,msg):
        self.updateFeedback()
        self.twist = self.controller.generateTwist(msg.twist.linear.x , msg.twist.angular.z)
        self.twist_pub.publish(self.twist)
        
    def updateFeedback(self):     
        try:
            (position,heading) = self.__listen.lookupTransform( self.frame_id,self.world_frame,rospy.Time(0)) 
            (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(heading)
            self.controller.setFeedback( Vector(position[0],position[1]), Vector(math.cos(yaw), math.sin(yaw)))
        except (tf.LookupException, tf.ConnectivityException),err:
            rospy.loginfo("could not locate vehicle")            


if __name__ == '__main__':
    rospy.init_node('cmd_vel_controller')
    node = Controller()
    reconfigure_server = Server(ParametersConfig, node.reconfigure_cb)
    rospy.spin()
    



    