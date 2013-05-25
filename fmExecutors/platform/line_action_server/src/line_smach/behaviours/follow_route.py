import rospy
import smach
import smach_ros
import actionlib
from line_smach.states import get_next_line
from line_action_server import *
from line_action_server.msg import *

def build(point_list):
    behaviour = smach.StateMachine(outcomes=['success','preempted','aborted'])
    with behaviour :
        smach.StateMachine.add('GET_NEXT', get_next_line.getNextLine(point_list), 
                               transitions={'succeeded':'GO_TO_POINT', 'aborted':'aborted'})                    
        smach.StateMachine.add('GO_TO_POINT', 
                               smach_ros.SimpleActionState('/fmExecutors/lineActionServer', lineAction, goal_slots=['a_x','a_y','b_x','b_y']),
                               transitions={'succeeded':'GET_NEXT','preempted':'preempted','aborted':'aborted'},
                               remapping={'a_x':'next_ax','a_y':'next_ay','b_x':'next_bx','b_y':'next_by'})
        
    return behaviour