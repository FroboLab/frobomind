#!/usr/bin/env python
import rospy
import smach
import smach_ros
import actionlib
import threading
import wii_remote
from wii_interface.wii_interface import * 
           
class mission():
    """    
        Top level user interface node implemented as a concurrence between wiimote interface and behaviour updater
    """
    def __init__(self):
        rospy.init_node('mission_control')

        self.hmi = WiiInterface()
            
        # Build the top level mission control from the remote control state and the autonomous state
        self.mission_control = smach.StateMachine(outcomes=['preempted'])            
        with self.mission_control:
            wii_remote.build_wii_remote(self.hmi)
            
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
        node = mission()
        smach_thread = threading.Thread(target = node.spin)
        smach_thread.start()
    except rospy.ROSInterruptException:
        pass