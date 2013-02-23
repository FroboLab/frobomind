import smach
import smach_ros
from wii_interface import wii_interface
from generic_smach.states import wii_states

def build(hmi):
    behaviour = smach.StateMachine(outcomes=['preempted'])            
    with behaviour:
       smach.StateMachine.add('REMOTE_CONTROL', wii_states.remoteControlState(hmi), transitions={'enterAutomode':'REMOTE_CONTROL','preempted':'preempted'})
    return behaviour