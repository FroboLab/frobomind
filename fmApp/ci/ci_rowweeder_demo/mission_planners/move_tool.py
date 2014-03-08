#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import threading
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8

from sdu_vibro_crop.msg import move_tool_simpleAction, move_tool_simpleGoal

def force_preempt(a):
    return True

def up_pressed(ud,msg):
    if msg.buttons[4]:
        return False
    else:
        return True

def down_pressed(ud,msg):
    if msg.buttons[5]:
        return False
    else:
        return True
    
def up_requested(ud,msg):
    if msg.data == 1:
        return False
    else:
        return True

def down_requested(ud,msg):
    if msg.data == 2:
        return False
    else:
        return True
    

def build_raise_lower_boom():
    btn_monitors = smach.Concurrence(outcomes=['raise','lower','raise_auto','lower_auto','succeeded'],
                                     default_outcome='succeeded',
                           outcome_map={
                                        "raise":{'MOVE_UP_BTN':'invalid','MOVE_DOWN_BTN':'preempted'}, 
                                        "lower":{'MOVE_UP_BTN':'preempted','MOVE_DOWN_BTN':'invalid'},
                                        "raise_auto":{"MOVE_UP_AUTO":"invalid"},
                                        "lower_auto":{"MOVE_DOWN_AUTO":"invalid"},
                                        },
                           child_termination_cb=force_preempt)
    
    with btn_monitors:
        smach.Concurrence.add("MOVE_UP_BTN", smach_ros.MonitorState("/fmLib/joy", Joy, up_pressed, -1))
        smach.Concurrence.add("MOVE_DOWN_BTN", smach_ros.MonitorState("/fmLib/joy", Joy, down_pressed, -1))
        smach.Concurrence.add("MOVE_UP_AUTO", smach_ros.MonitorState("/fmData/boom_state",Int8,up_requested, -1))
        smach.Concurrence.add("MOVE_DOWN_AUTO", smach_ros.MonitorState("/fmData/boom_state",Int8,down_requested, -1))
        
    sm = smach.StateMachine(outcomes=["succeeded","aborted","preempted"])
    
    with sm:
        smach.StateMachine.add("MONITOR_BUTTONS", 
                               btn_monitors, 
                               transitions={"raise":"MOVE_UP",
                                            "lower":"MOVE_DOWN",
                                            "succeeded":"MONITOR_BUTTONS",
                                            "raise_auto":"MOVE_UP",
                                            "lower_auto":"MOVE_DOWN"})
        smach.StateMachine.add("MOVE_UP",
                               smach_ros.SimpleActionState("/fmImplements/RowClean/move_tool",move_tool_simpleAction,goal=move_tool_simpleGoal(direction=1,timeout=12)),
                               transitions = {"succeeded":"MONITOR_BUTTONS","aborted":"aborted","preempted":"preempted"}
                               )
        smach.StateMachine.add("MOVE_DOWN",
                               smach_ros.SimpleActionState("/fmImplements/RowClean/move_tool",move_tool_simpleAction,goal=move_tool_simpleGoal(direction=0,timeout=12)),
                               transitions = {"succeeded":"MONITOR_BUTTONS","aborted":"aborted","preempted":"preempted"}
                               )
    
    return sm

if __name__ == "__main__":
    try:
        rospy.init_node('mission_control')
        sm = build_raise_lower_boom() 
        sis = smach_ros.IntrospectionServer('StateMachineView', sm, '/SM_ROOT')           
        sis.start()    
        sm.execute()
        rospy.spin()
    except rospy.ROSInterruptException:
        sm.request_preempt()
        sis.stop()