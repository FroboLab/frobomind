import rospy
import smach
import smach_ros
from action_primitives.msg import *
from demining_smach.states import get_step_towards_point
from position_action_server import *
from position_action_server.msg import *
from std_msgs.msg import Float64

def build():
    behaviour = smach.StateMachine(outcomes=['inspectionDone','preempted','aborted'], input_keys=['next_x', 'next_y'])
    # Wrigle goals
    wriggle_left_goal = make_turnGoal(amount=1 , vel=0.3, forward_vel=0)
    wriggle_right_goal = make_turnGoal(amount=-2, vel=0.3, forward_vel=0)
    wriggle_center_goal = make_turnGoal(amount=1, vel=0.3, forward_vel=0)

    with behaviour:
         smach.StateMachine.add('get_next_search_point', get_step_towards_point.getStepTowardsPoint(),transitions={'succeeded':'wriggle_left'})
         smach.StateMachine.add('wriggle_left', smach_ros.SimpleActionState('/fmExecutors/make_turn', make_turnAction, wriggle_left_goal), transitions={'succeeded':'wriggle_right','preempted':'preempted','aborted':'aborted'})
         smach.StateMachine.add('wriggle_right', smach_ros.SimpleActionState('/fmExecutors/make_turn', make_turnAction, wriggle_right_goal), transitions={'succeeded':'wriggle_center','preempted':'preempted','aborted':'aborted'})
         smach.StateMachine.add('wriggle_center', smach_ros.SimpleActionState('/fmExecutors/make_turn', make_turnAction, wriggle_center_goal), transitions={'succeeded':'go_to_next_search_point','preempted':'preempted','aborted':'aborted'})
         smach.StateMachine.add('go_to_next_search_point', smach_ros.SimpleActionState('/fmExecutors/position_planner',positionAction, goal_slots=['x','y']),
                               transitions={'succeeded':'check_no_mines','preempted':'preempted','aborted':'aborted'},
                               remapping={'x':'step_next_x','y':'step_next_y'})
         smach.StateMachine.add('check_no_mines', smach_ros.MonitorState("/wads", Float64, mine_detect_cb, 1), transitions={'valid':'inspectionDone', 'invalid':'get_next_search_point', 'preempted':'preempted'})
        
    return behaviour

def mine_detect_cb(userdata, msg):
    if (msg.data > 4):
        rospy.loginfo("Found mine")
        return False # Mine found
    else:
        rospy.loginfo("No mines found")
        return True # No mines found. Continue coverage
