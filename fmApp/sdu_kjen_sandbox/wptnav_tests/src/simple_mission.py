#!/usr/bin/env python
#/****************************************************************************
# FroboMind
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
import rospy, smach, smach_ros
from generic_smach.states import wii_states
from wii_interface import wii_interface

class simpleAutomode(smach.State):
    """
        Implementation of a simple automode
    """
    
    def __init__(self, hmi):
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'])
        self.hmi = hmi

    def execute(self, userdata):
        print("You are now in automode")
        self.r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.hmi.automode :
                # Publish topics
                try :
                    self.r.sleep()
                except rospy.ROSInterruptException:
                    return 'preempted'
            else :
                return  'succeeded'
        return 'preempted' 

           
class mission():
    """    
        Top level user interface node implemented as a concurrence between wiimote interface and behaviour updater
    """
    def __init__(self):
        rospy.init_node('mission_control')
        self.hmi = wii_interface.WiiInterface()
        self.hmi.register_callback_button_A(self.onButtonA)
        
    def build_auto(self):
        """    
            Builds the mission behaviour
        """
         # Build the autonomous state as concurrence between wiimote and measuring behaviour to allow user preemption
        autonomous = smach.Concurrence(outcomes = ['exitAutomode','aborted'], default_outcome = 'exitAutomode', outcome_map = {'exitAutomode':{'HMI':'preempted','USER_CODE':'preempted'}, 'aborted':{'HMI':'preempted','USER_CODE':'aborted'}}, child_termination_cb = onPreempt)
        with autonomous:
            smach.Concurrence.add('HMI', wii_states.interfaceState(self.hmi))
            smach.Concurrence.add('USER_CODE',  self.build_usercode())
        
        # Build the top level mission control from the remote control state and the autonomous state
        mission_control = smach.StateMachine(outcomes=['preempted','aborted'])            
        with mission_control:
            smach.StateMachine.add('REMOTE_CONTROL', wii_states.remoteControlState(self.hmi), transitions={'enterAutomode':'AUTO_MODE','preempted':'preempted'})
            smach.StateMachine.add('AUTO_MODE', autonomous, transitions={'exitAutomode':'REMOTE_CONTROL','aborted':'REMOTE_CONTROL'})
        return mission_control

    def build_usercode(self):
        behaviour = smach.StateMachine(outcomes=['succeeded','preempted','aborted'])
        with behaviour :
            smach.StateMachine.add('USER', simpleAutomode(self.hmi), \
				transitions={'succeeded':'succeeded', 'aborted':'aborted'})                    
        return behaviour
    
    def spin(self): 
        self.sm = self.build_auto()   
        self.sis = smach_ros.IntrospectionServer('StateMachineView', self.sm, '/SM_ROOT')           
        self.sis.start() 
        self.sm.execute()
        rospy.spin()   
        
    def quit(self):
        self.sm.request_preempt()
        self.sis.stop()
        
    def onButtonA(self):
		pass
 
def onPreempt(outcome_map):
    """
        Preempts all other states on child termination. 
    """
    return True 
    
if __name__ == '__main__':
    try:
        node = mission()
        node.spin()
    except rospy.ROSInterruptException:
        node.quit()
