#/****************************************************************************
# FroboMind wii_interface.py
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
import rospy
from std_msgs.msg import Char, Bool
from geometry_msgs.msg import TwistStamped

class KeyInterface():
    """
        Keyboard interface.
    """
    def __init__(self):
        # Setup parameters
        self.automode = False
        self.twist = TwistStamped()
        self.deadman = Bool()
        self.deadman.data = True

        # Get parameters
        self.max_linear_velocity = rospy.get_param("~max_linear_velocity",2)
        self.max_angular_velocity = rospy.get_param("~max_angular_velocity",4)
        self.publish_frequency = rospy.get_param("~publish_frequency",10)
        self.step_size = rospy.get_param("~step_size",0.5)    
        # Get topic names
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic",'cmd_vel')
        self.deadman_topic = rospy.get_param("~deadman_topic",'deadman')
        self.key_topic = rospy.get_param("~key_topic",'keyboard')
        # Setup topics
        self.twist_pub = rospy.Publisher(self.cmd_vel_topic, TwistStamped)
        self.deadman_pub = rospy.Publisher(self.deadman_topic, Bool)
        self.joy_sub = rospy.Subscriber(self.key_topic, Char, self.onKey )
        
    def onKey(self,msg):
        if msg.data == int('0x43',16) : # Right
            self.twist.twist.angular.z = self.twist.twist.angular.z - self.step_size
        if msg.data == int('0x44',16) : # Left
            self.twist.twist.angular.z = self.twist.twist.angular.z + self.step_size
        if msg.data == int('0x41',16) : # Up
            self.twist.twist.linear.x = self.twist.twist.linear.x + self.step_size
        if msg.data == int('0x42',16) : # Down
            self.twist.twist.linear.x = self.twist.twist.linear.x - self.step_size
                        
    def publishCmdVel(self): 
        """
            Method to average and publish twist from wiimote input
        """
        self.deadman_pub.publish(self.deadman)
        # Publish twist message                   
        self.twist.header.stamp = rospy.Time.now()
        self.twist_pub.publish(self.twist)
        
   
    