from wii_interface.wii_interface import *

def build_wii_remote(hmi):
    return smach.StateMachine.add('REMOTE_CONTROL', remoteControlState(hmi), transitions={'enterAutomode':'REMOTE_CONTROL','preempted':'preempted'})
