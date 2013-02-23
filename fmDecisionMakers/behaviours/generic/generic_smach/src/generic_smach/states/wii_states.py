#!/usr/bin/env python
import rospy
import smach
import smach_ros

class remoteControlState(smach.State):
    """
        Implements remote controlled operation as a smach state
        Usage example:
            smach.StateMachine.add('REMOTE_CONTROL', remoteControlState(self.hmi), transitions={'enterAutomode':'AUTO_MODE'})
    """
    def __init__(self,hmi):
        smach.State.__init__(self, outcomes=['enterAutomode','preempted'])
        # Instantiate HMI
        self.hmi = hmi

    def execute(self, userdata):
        self.r = rospy.Rate(self.hmi.publish_frequency)
#        while not self.hmi.automode and not rospy.is_shutdown():
        while not rospy.is_shutdown():
            if not self.hmi.automode :
                # Publish topics
                self.hmi.publishDeadman()
                self.hmi.publishFeedback()
                self.hmi.publishCmdVel()
                # Spin
                self.r.sleep()
            else :
                return  'enterAutomode'
        return 'preempted'
        
class interfaceState(smach.State):
    """
        Implements HMI as smach state to be used in concurrence with autonomous behaviours
        Usage example:
            smach.Concurrence.add('HMI', interfaceState(self.hmi))
    """
    def __init__(self,hmi):
        smach.State.__init__(self, outcomes=['preempted'])
        # Instantiate HMI
        self.hmi = hmi
        self.r = rospy.Rate(self.hmi.publish_frequency)
        
    def execute(self, userdata):
        while not rospy.is_shutdown():         
            if self.hmi.automode :
                # Publish topics
                self.hmi.publishDeadman()
                self.hmi.publishFeedback()
                self.r.sleep()
            else :
                break
        return 'preempted'