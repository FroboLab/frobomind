#!/usr/bin/env python
#/****************************************************************************
# FroboMind remote.py
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
import rospy
import smach
import smach_ros
import actionlib
import threading
from generic_smach.behaviours import remote_behaviour
from wii_interface import wii_interface
from sdu_vibro_crop.msg import *
           
class mission():
    """    
        Top level user interface node implemented as a concurrence between wiimote interface and behaviour updater
    """
    def __init__(self):
        rospy.init_node('mission_control')
        self.hmi = wii_interface.WiiInterface()
        self.hmi.register_callback_button_A(self.onButtonA)
        self.hmi.register_callback_button_up(self.onButtonUp)
        self.hmi.register_callback_button_down(self.onButtonDown)
        self.client = actionlib.SimpleActionClient('kongskilde_row_cleaner/move_tool', move_tool_simpleAction)
        self.goal = move_tool_simpleGoal()
        self.implement_state = 0
        
    def spin(self): 
        self.client.wait_for_server()  
        sm = remote_behaviour.build(self.hmi) 
        sis = smach_ros.IntrospectionServer('StateMachineView', sm, '/SM_ROOT')           
        sis.start()    
        sm.execute()
        rospy.spin()
        sm.request_preempt()
        sis.stop()
        
    def onButtonA(self):
        rospy.loginfo("A pressed")
        self.implement_state = 0
        
    def onButtonUp(self):
        if not self.implement_state == 2  :
            rospy.loginfo("Raising implement")
            self.goal.direction = 0
            self.goal.timeout = 10
            self.client.send_goal(self.goal)
            self.implement_state = 2
        
    def onButtonDown(self): 
        if not self.implement_state == 1 :
            rospy.loginfo("Lowering implement")    
            self.goal.direction = 1
            self.goal.timeout = 10
            self.client.send_goal(self.goal)
            self.implement_state = 1
    
if __name__ == '__main__':
    try:
        node = mission()
        smach_thread = threading.Thread(target = node.spin)
        smach_thread.start()
    except rospy.ROSInterruptException:
        pass