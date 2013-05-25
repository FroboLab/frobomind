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
import rospy, tf, math, numpy
from nav_msgs.msg import Odometry

class Printer():
    """
        Converter 
    """
    def __init__(self):
        # Init node
        self.rate = rospy.Rate(5)     
        self.odom_frame = rospy.get_param("~odom_frame","/world")
        self.base_frame = rospy.get_param("~base_frame","/base_footprint")
        self.odometry_topic = rospy.get_param("~odometry_topic","/fmKnowledge/pose")
        self.use_tf = rospy.get_param("~use_tf",False)
        self.use_tf = False
        self.quaternion = numpy.empty((4, ), dtype=numpy.float64)
        
        if self.use_tf :
            self.__listen = tf.TransformListener()
        else :
            self.odom_sub = rospy.Subscriber(self.odometry_topic, Odometry, self.onOdometry )
            

    def spin(self):
        rospy.sleep(1)
        while not rospy.is_shutdown() :
            self.get_current_position()

            try :
                self.rate.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo("Interrupted during sleep")
                return 'preempted'


    def get_current_position(self):
        """
            Get current position from tf
        """
        if self.use_tf :
            try:
                (position,head) = self.__listen.lookupTransform( self.odom_frame,self.base_frame,rospy.Time(0)) # The transform is returned as position (x,y,z) and an orientation quaternion (x,y,z,w).
                (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(head)
                rospy.loginfo("State(x,y,yaw): (" + str(position[0]) + " , " + str(position[1]) + " , " + str(yaw*180/math.pi) + ")")
            except (tf.LookupException, tf.ConnectivityException),err:
                rospy.loginfo("could not locate vehicle - " + str(err)) 
            
    def onOdometry(self, msg):
        """
            Callback method for handling odometry messages
        """
        # Extract the orientation quaternion
        self.quaternion[0] = msg.pose.pose.orientation.x
        self.quaternion[1] = msg.pose.pose.orientation.y
        self.quaternion[2] = msg.pose.pose.orientation.z
        self.quaternion[3] = msg.pose.pose.orientation.w
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(self.quaternion)
        rospy.loginfo("State(x,y,yaw): (" + str(msg.pose.pose.position.x) + " , " + str(msg.pose.pose.position.y) + " , " + str(yaw*180/math.pi) + ")")
            

if __name__ == '__main__':
    rospy.init_node('state_printer')
    node = Printer()
    node.spin()
    



    