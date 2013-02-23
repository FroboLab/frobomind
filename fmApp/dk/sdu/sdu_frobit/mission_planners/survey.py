#!/usr/bin/env python
import rospy
import smach
import smach_ros
import actionlib
import threading
from wii_interface import wii_interface 
from generic_smach.states import wait_state,wii_states
from position_action_server import *
from position_action_server.msg import *
from nav_msgs.msg import Odometry

class getNextPosition(smach.State):
    """
        Temporary implementation. State giving the next position goal
        TODO: Must be implemented as a service interfacing to a file
    """
    def __init__(self,next_position):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.next_position = next_position
        self.ptr = 0
        self.position_list =[[2,2],[-3,-3],[1,-1],[-2,2],[2,-4]]

    def execute(self, userdata):
        self.next_position.x = self.position_list[self.ptr][0]
        self.next_position.y = self.position_list[self.ptr][1]
        self.ptr = self.ptr + 1
        if self.ptr == len(self.position_list) - 1 :
            self.ptr = 0
        return 'succeeded'       
  
   
           
class pichiSurveyMission():
    """    
        Top level user interface node implemented as a concurrence between wiimote interface and behaviour updater
    """
    def __init__(self):
        rospy.init_node('mission_control')

        self.hmi = wii_interface.WiiInterface()
        self.next_position = positionGoal(x=0,y=0)
        
        # Build the measuring behaviour.
        # For now implemented here for convenience. Must implement user data passing and move to behaviours
        measuring_behaviour = smach.StateMachine(outcomes=['success','preempted','aborted'])
        with measuring_behaviour :
            smach.StateMachine.add('GET_NEXT', getNextPosition(self.next_position), 
                                   transitions={'succeeded':'GO_TO_POINT'})                    
            smach.StateMachine.add('GO_TO_POINT', 
                                   smach_ros.SimpleActionState('/positionActionServer',positionAction,goal=self.next_position),
                                   transitions={'succeeded':'MEASURE','preempted':'preempted','aborted':'aborted'})
            smach.StateMachine.add('MEASURE' , wait_state.WaitState(3),
                                   transitions={'succeeded':'GET_NEXT','preempted':'preempted'})

        # Build the autonomous state as concurrence between wiimote and measuring behaviour to allow user preemption
        autonomous = smach.Concurrence(  outcomes = ['exitAutomode'],
                                                default_outcome = 'exitAutomode',
                                                outcome_map = {'exitAutomode':{'HMI':'preempted','MEASURE':'preempted'}},
                                                child_termination_cb = onPreempt)
        with autonomous:
            smach.Concurrence.add('HMI', wii_states.interfaceState(self.hmi))
            smach.Concurrence.add('MEASURE', measuring_behaviour)
            
        # Build the top level mission control from the remote control state and the autonomous state
        self.mission_control = smach.StateMachine(outcomes=['preempted'])            
        with self.mission_control:
            smach.StateMachine.add('REMOTE_CONTROL', wii_states.remoteControlState(self.hmi), transitions={'enterAutomode':'AUTO_MODE','preempted':'preempted'})
            smach.StateMachine.add('AUTO_MODE', autonomous, transitions={'exitAutomode':'REMOTE_CONTROL'})

        sis = smach_ros.IntrospectionServer('StateMachineView', self.mission_control, '/SM_ROOT')           
        sis.start()
       
    def spin(self):        
        self.mission_control.execute()
        rospy.spin()
        sis.stop()

def onPreempt(outcome_map):
    """
        Preempts all other states on child termination. 
        TODO: Find a way to avoid this being a global function...
    """
    return True
    
if __name__ == '__main__':
    try:
        node = pichiSurveyMission()
        smach_thread = threading.Thread(target = node.spin)
        smach_thread.start()
    except rospy.ROSInterruptException:
        pass