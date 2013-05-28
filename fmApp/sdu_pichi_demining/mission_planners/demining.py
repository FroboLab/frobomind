#!/usr/bin/env python
#/****************************************************************************
# FroboMind demiming.py
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
from wii_interface import wii_interface 
from demining_smach.behaviours import follow_route
from demining_smach.behaviours import mine_simple_avoidance
from generic_smach.states import wii_states
from nav_msgs.msg import Odometry    
from std_msgs.msg import Float64

# Simple test to delay the wads moitor and avoid looping
#class wadsPenalty(smach.State):
#     def __init__(self, outcomes=['done']):
#         
#     def execute(self, userdata):
#         rospy.sleep(1)
#         return 'done'

class Mission():
    """    
        Top level user interface node implemented as a concurrence between wiimote interface and behaviour updater
    """
    def __init__(self):
        rospy.init_node('mission_control')
        rospy.loginfo("mission control initialized")
        self.hmi = wii_interface.WiiInterface()
        self.hmi.register_callback_button_A(self.onButtonA)
          
    def build(self):
        # Build concurrent route following and mine detection
        mine_search = smach.Concurrence ( outcomes = ['mineDetected','preempted'], 
                                       default_outcome = 'preempted',
                                       outcome_map = {'preempted':{'follow_route':'preempted'}, 'mineDetected':{'mine_detect':'invalid'}},
                                       output_keys=['next_x','next_y'],
                                       child_termination_cb = onPreempt)

        with mine_search:
            smach.Concurrence.add('follow_route', follow_route.build())
            smach.Concurrence.add('mine_detect', smach_ros.MonitorState("/wads", Float64, mine_detect_cb))

        # Build the demining task        
        demining = smach.StateMachine(outcomes=['success', 'preempted', 'aborted'])
        with demining:
            smach.StateMachine.add('mine_search', mine_search, transitions={'mineDetected':'mine_inspection'})
            smach.StateMachine.add('mine_inspection', mine_simple_avoidance.build() , transitions={'inspectionDone':'mine_search'})

        # Build the autonomous state as concurrence between wiimote and measuring behaviour to allow user preemption
        autonomous = smach.Concurrence(  outcomes = ['exitAutomode'],
                                                default_outcome = 'exitAutomode',
                                                outcome_map = {'exitAutomode':{'HMI':'preempted','demining':'preempted'}},
                                                child_termination_cb = onPreempt)

        with autonomous:
            smach.Concurrence.add('HMI', wii_states.interfaceState(self.hmi))
            smach.Concurrence.add('demining', demining)


        
        # Build the top level mission control from the remote control state and the autonomous state
        mission_control = smach.StateMachine(outcomes=['preempted'])            
        with mission_control:
            smach.StateMachine.add('REMOTE_CONTROL', wii_states.remoteControlState(self.hmi), transitions={'enterAutomode':'AUTO_MODE','preempted':'preempted'})
            smach.StateMachine.add('AUTO_MODE', autonomous, transitions={'exitAutomode':'REMOTE_CONTROL'})
        return mission_control
                       
    def spin(self):    
        self.sm = self.build()   
        self.sis = smach_ros.IntrospectionServer('StateMachineView', self.sm, '/SM_ROOT')           
        self.sis.start() 
        self.sm.execute()
        rospy.spin()

    def quit(self):
        sis.stop()        

    def onButtonA(self):
        rospy.loginfo("A pressed")

def mine_detect_cb(userdata, msg):
    if (msg.data > 1):
        return False # terminate
    else:
        return True # Continue searching for mines

def onPreempt(outcome_map):
    """
        Preempts all other states on child termination. 
        TODO: Find a way to avoid this being a global function...
    """
    return True
    
if __name__ == '__main__':
    try:
        node = Mission()
        node.spin()
    except rospy.ROSInterruptException:
        node.quit()
