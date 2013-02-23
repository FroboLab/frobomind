#!/usr/bin/env python
import rospy
import smach
import smach_ros
import actionlib
import threading
from wii_interface import wii_interface 
from surveyor_smach.behaviours import measure_point
from generic_smach.states import wii_states
from nav_msgs.msg import Odometry    
           
class Mission():
    """    
        Top level user interface node implemented as a concurrence between wiimote interface and behaviour updater
    """
    def __init__(self):
        rospy.init_node('mission_control')
        self.hmi = wii_interface.WiiInterface()
          
    def build(self):
         # Build the autonomous state as concurrence between wiimote and measuring behaviour to allow user preemption
        autonomous = smach.Concurrence(  outcomes = ['exitAutomode'],
                                                default_outcome = 'exitAutomode',
                                                outcome_map = {'exitAutomode':{'HMI':'preempted','MEASURE':'preempted'}},
                                                child_termination_cb = onPreempt)
        with autonomous:
            smach.Concurrence.add('HMI', wii_states.interfaceState(self.hmi))
            smach.Concurrence.add('MEASURE', measure_point.build())
        
        # Build the top level mission control from the remote control state and the autonomous state
        mission_control = smach.StateMachine(outcomes=['preempted'])            
        with mission_control:
            smach.StateMachine.add('REMOTE_CONTROL', wii_states.remoteControlState(self.hmi), transitions={'enterAutomode':'AUTO_MODE','preempted':'preempted'})
            smach.StateMachine.add('AUTO_MODE', autonomous, transitions={'exitAutomode':'REMOTE_CONTROL'})
        return mission_control
                       
    def spin(self):    
        sm = self.build()   
        sis = smach_ros.IntrospectionServer('StateMachineView', sm, '/SM_ROOT')           
        sis.start() 
        sm.execute()
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
        node = Mission()
        smach_thread = threading.Thread(target = node.spin)
        smach_thread.start()
    except rospy.ROSInterruptException:
        pass