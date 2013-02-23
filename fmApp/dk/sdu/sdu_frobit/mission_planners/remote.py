#!/usr/bin/env python
import rospy
import smach
import smach_ros
import actionlib
import threading
from generic_smach.behaviours import remote_behaviour
from wii_interface import wii_interface
           
class mission():
    """    
        Top level user interface node implemented as a concurrence between wiimote interface and behaviour updater
    """
    def __init__(self):
        rospy.init_node('mission_control')
        self.hmi = wii_interface.WiiInterface()
       
    def spin(self):   
        sm = remote_behaviour.build(self.hmi) 
        sis = smach_ros.IntrospectionServer('StateMachineView', sm, '/SM_ROOT')           
        sis.start()    
        sm.execute()
        rospy.spin()
        sm.request_preempt()
        sis.stop()
    
if __name__ == '__main__':
    try:
        node = mission()
        smach_thread = threading.Thread(target = node.spin)
        smach_thread.start()
    except rospy.ROSInterruptException:
        pass