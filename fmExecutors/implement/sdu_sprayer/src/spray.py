#/****************************************************************************
# FroboMind template_cpp_node
# Copyright (c) 2011-2013, author Kent Stark Olsen kent.stark.olsen@gmail.com
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#	* Redistributions of source code must retain the above copyright
#  	notice, this list of conditions and the following disclaimer.
#	* Redistributions in binary form must reproduce the above copyright
#  	notice, this list of conditions and the following disclaimer in the
#  	documentation and/or other materials provided with the distribution.
#	* Neither the name FroboMind nor the
#  	names of its contributors may be used to endorse or promote products
#  	derived from this software without specific prior written permission.
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

#!/usr/bin/env python
import roslib; 
roslib.load_manifest("fmExecutors")
import rospy

import actionlib
import math
import tf

from tf import TransformListener, TransformBroadcaster
from geometry_msgs.msg import Twist

from fmExecutors.msg import *
from std_msgs.msg import UInt32 


class SprayAction():
    """
        Drives x meters either forward or backwards depending on the given distance.
        the velocity should always be positive. 
    """
    def __init__(self,name,odom_frame,base_frame):
        """
        
        @param name: the name of the action
        @param odom_frame: the frame the robot is moving in (odom_combined)
        @param base_frame: the vehicles own frame (usually base_link)
        """
        
        self._action_name = name
        self.__odom_frame = odom_frame
        self.__base_frame = base_frame
        self.__server =  actionlib.SimpleActionServer(self._action_name,sprayAction,auto_start=False)
        self.__server.register_preempt_callback(self.preempt_cb)
        self.__server.register_goal_callback(self.goal_cb)
        
        self.__cur_pos = 0
        self.__start_pos = 0
        self.__start_yaw = 0
        self.__cur_yaw = 0
        
        self.__feedback = sprayFeedback()
        
        self.__listen = TransformListener()
        #self.vel_pub = rospy.Publisher("/fmControllers/cmd_vel_auto",Twist)
        self.spray_pub = rospy.Publisher("/fmControllers/spray",UInt32)
        
        self.__start_time = rospy.Time.now()
        self.new_goal = False
        self.n_sprays = 0
        self.spray_msg = UInt32()
        
        self.spray_l = False
        self.spray_cnt = 0
        
        self.__server.start()

    def preempt_cb(self):
        rospy.loginfo("Preempt requested")
        self.spray_msg.data = 0;
        self.spray_pub.publish(self.spray_msg)
        self.__server.set_preempted()
    
    def goal_cb(self):
        """
            called when we receive a new goal
            the goal contains a desired radius and a success radius in which we check if the turn succeeded or not
            the message also contains if we should turn left or right
        """
        g = self.__server.accept_new_goal()
        self.spray_dist= g.distance
        self.spraytime = g.spray_time
        
        self.new_goal = True
    
    def on_timer(self,e):
        """
            called regularly by a ros timer
            in here we simply orders the vehicle to start moving either forward 
            or backwards depending on the distance sign, while monitoring the 
            travelled distance, if the distance is equal to or greater then this
            action succeeds.
        """
        if self.__server.is_active():
            if self.new_goal:
                self.new_goal = False
                self.n_sprays = 0
                self.__get_start_position()
            else:
                if self.__get_current_position():
                    if self.__get_distance() >= self.spray_dist:
                        self.do_spray()
                        self.__get_start_position()
                else:
                    self.__server.set_aborted(None, "could not locate")
                    rospy.logerr("Could not locate vehicle")
                    self.spray_msg.data = 0;
                    self.spray_pub.publish(self.spray_msg)
                            
    def __get_distance(self):
        return math.sqrt(math.pow(self.__cur_pos[0][0] - self.__start_pos[0][0],2) +
                         math.pow(self.__cur_pos[0][1] - self.__start_pos[0][1],2))
    
    def __get_start_position(self):
        ret = False
        try:
            self.__start_pos = self.__listen.lookupTransform(self.__odom_frame,self.__base_frame,rospy.Time(0))
            self.__start_yaw = tf.transformations.euler_from_quaternion(self.__start_pos[1])[2]
            ret = True
        except (tf.LookupException, tf.ConnectivityException),err:
            rospy.loginfo("could not locate vehicle")
        
        return ret
    
    def __get_current_position(self):
        ret = False
        try:
            self.__cur_pos = self.__listen.lookupTransform(self.__odom_frame,self.__base_frame,rospy.Time(0))
            self.__cur_yaw = tf.transformations.euler_from_quaternion(self.__cur_pos[1])[2]
            ret = True
        except (tf.LookupException, tf.ConnectivityException),err:
            rospy.loginfo("could not locate vehicle")
        
        return ret
    
    def do_spray(self):
        if self.spray_cnt >= 2:
            self.spray_cnt = 0
            self.spray_l = not self.spray_l
            
        if self.spray_l == True:
            self.spray_msg.data = 1
            self.spray_pub.publish(self.spray_msg)
            self.spray_cnt += 1
         
        if self.spray_l == False:
            self.spray_msg.data = 2
            self.spray_pub.publish(self.spray_msg)
            self.spray_cnt += 1
            
        self.n_sprays += 1
        self.__feedback.cur_sprays = self.n_sprays
        self.__server.publish_feedback(self.__feedback)

if __name__ == "__main__":
    
    rospy.init_node("spray")
    
    action_server = SprayAction("spray","odom_combined","base_footprint")
    
    t = rospy.Timer(rospy.Duration(0.05),action_server.on_timer)
    
    rospy.spin()
    
