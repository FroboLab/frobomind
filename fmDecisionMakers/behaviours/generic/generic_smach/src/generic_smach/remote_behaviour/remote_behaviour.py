import smach
import smach_ros
from wii_interface import wii_interface
from generic_smach.wii_states import wii_states

def build(hmi):
    return smach.StateMachine.add('REMOTE_CONTROL', wii_states.remoteControlState(hmi), transitions={'enterAutomode':'REMOTE_CONTROL','preempted':'preempted'})
