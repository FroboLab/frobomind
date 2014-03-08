#!/usr/bin/env python
#/****************************************************************************
# FroboMind survey.py
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
# Change log:
# 25-Mar 2013 Leon: Changed fixed list to recorded list (button A adds point)
# 23-Apr 2013 Leon: Updated to use tf instead of odom topic
#
#****************************************************************************/
import rospy,smach,smach_ros,actionlib,threading,tf
from wii_interface import wii_interface 
from surveyor_smach.behaviours import measure_point
from generic_smach.states import wii_states
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Point   
           
class Mission():
    """    
        Top level user interface node implemented as a concurrence between wiimote interface and 
        the mission behaviour
    """
    def __init__(self):
        rospy.init_node('mission_control')
        rospy.loginfo("mission control initialized")
        
        self.odom_topic = rospy.get_param("~odom_topic",'/odom')
        self.tf = tf.TransformListener()
        #self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.onOdometry )
        
        self.hmi = wii_interface.WiiInterface()
        rospy.loginfo("Registering save point callback")
        self.hmi.register_callback_button_A(self.savePoint)
        self.point_list = [Point(4,4,0) , Point(5,7,0) , Point(1,2,0), Point(2,2,0)]
        self.latest_point = Point()
        self.save_time = rospy.Time.now()
        self.min_time_between_point_save = rospy.Duration(3) # No magic numbers...
          
    def build(self):
        """    
            Builds the mission behaviour
        """
         # Build the autonomous state as concurrence between wiimote and measuring behaviour to allow user preemption
        autonomous = smach.Concurrence(  outcomes = ['exitAutomode','aborted'],
                                                default_outcome = 'exitAutomode',
                                                outcome_map = {'exitAutomode':{'HMI':'preempted','MEASURE':'preempted'},
                                                               'aborted':{'HMI':'preempted','MEASURE':'aborted'}},
                                                child_termination_cb = onPreempt)
        with autonomous:
            smach.Concurrence.add('HMI', wii_states.interfaceState(self.hmi))
            smach.Concurrence.add('MEASURE', measure_point.build(self.point_list))
        
        # Build the top level mission control from the remote control state and the autonomous state
        mission_control = smach.StateMachine(outcomes=['preempted','aborted'])            
        with mission_control:
            smach.StateMachine.add('REMOTE_CONTROL', wii_states.remoteControlState(self.hmi), transitions={'enterAutomode':'AUTO_MODE','preempted':'preempted'})
            smach.StateMachine.add('AUTO_MODE', autonomous, transitions={'exitAutomode':'REMOTE_CONTROL','aborted':'REMOTE_CONTROL'})
        return mission_control
                       
    def spin(self):
        """    
            Method to spin the node
        """    
        self.sm = self.build()   
        self.sis = smach_ros.IntrospectionServer('StateMachineView', self.sm, '/SM_ROOT')           
        self.sis.start() 
        self.sm.execute()
        rospy.spin()       
        
    def savePoint(self):
        """    
            Callback method to save the current position
        """
#        if self.save_time < rospy.Time.now():
#            rospy.loginfo("Saving point: (%f,%f) ",self.latest_point.x,self.latest_point.y)
#            self.point_list.append(self.latest_point)
#            self.save_time = rospy.Time.now() + self.min_time_between_point_save
        if self.save_time < rospy.Time.now():
            try:
                (position,orientation) = self.tf.lookupTransform("world_frame" , "mast_bottom",rospy.Time(0)) 
                self.latest_point.x = position[0]
                self.latest_point.y = position[1]   
                rospy.loginfo("Saving point: (%f,%f) ",self.latest_point.x,self.latest_point.y)       
            except (tf.LookupException, tf.ConnectivityException, tf.Exception),err:
                rospy.loginfo("Transform error: %s",err)
        
    def onOdometry(self,msg):
        """    
            Callback method to save current position
        """
        self.latest_point = msg.pose.pose.position
    
    def quit(self):
        """    
            Method to properly turn down and avoid SIGTERM
        """
        self.sis.stop()
        self.sm.request_preempt()
        self.sm.close()

def onPreempt(outcome_map):
    """
        Preempts all other states on child termination. 
    """
    return True
    
if __name__ == '__main__':
    try:
        node = Mission()
        node.spin()
    except rospy.ROSInterruptException:
        node.quit()