#!/usr/bin/env python
import roslib; 

roslib.load_manifest("fmDecisionMakers")
import rospy

import behaviours

import actionlib
import math
import tf

from sensor_msgs.msg import Joy
import threading
import smach
import smach_ros

# behaviours used in this statemachine
from fmImplements.msg import move_tool_simpleAction, move_tool_simpleGoal
from fmExecutors.msg import follow_pathAction, follow_pathGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import behaviours.wii_states.wii_auto_manuel

    
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
    
    
def load_path(filename):
    path = Path()
    state = []
    with open(filename) as file:
        for line in file:
            if not line.startswith("#"):
                p = PoseStamped()
                x,y,st=line.split()
                p.header.frame_id = "odom_combined"
                p.pose.position.x = float(x)
                p.pose.position.y = float(y)
                state.append(st.lower())
                path.poses.append(p)
    
    return path,state

@smach.cb_interface(input_keys=['current_index_in'],
                    output_keys=['current_index_out'],
                    outcomes=['done'])
def on_preempted(ud):
    # rewind current index by one so that the current wp is resumed instead of the next
    ud.current_index_out = ud.current_index_in - 1
    
    return 'done'

@smach.cb_interface(input_keys=['max_index','path','current_index_in','boom_state_array'],
                    output_keys=['current_index_out','current_wp_out','direction_out'],
                    outcomes=['continue','update','done'])
def path_follow_cb(ud):
    ret = 'continue'
    if ud.current_index_in < (ud.max_index-1):
        
        if  ud.boom_state_array[ud.current_index_in] == "up":
            ud.direction_out = move_tool_simpleGoal.UP
        else:
            ud.direction_out = move_tool_simpleGoal.DOWN
        wp = follow_pathGoal()
        wp.path.poses.append(ud.path.poses[ud.current_index_in])
        wp.path.poses.append(ud.path.poses[ud.current_index_in + 1])
        ud.current_wp_out = wp.path
        

        
        if ud.current_index_in == 0:
            ret = 'update'
        elif ud.boom_state_array[ud.current_index_in] != ud.boom_state_array[ud.current_index_in-1]:
            ret = 'update'
        
        ud.current_index_out = ud.current_index_in + 1
        
        return ret
    else:
        return 'done'
    
    

def build_nav_sm(path,tool_state):
    """
        generates the statemachine which follows the given path and
        raises and lowers boom using the path indexes in raise_tool_states
    """
    
    nav_sm = smach.StateMachine(outcomes=["succeeded","aborted","preempted"])
    
    nav_sm.userdata.max_index = len(path.poses) - 1
    nav_sm.userdata.path = path
    nav_sm.userdata.current_index = 0
    nav_sm.userdata.boom_state_array = tool_state
    nav_sm.userdata.direction = 0
    nav_sm.userdata.timeout = 7
    
    
    with nav_sm:
        smach.StateMachine.add("SELECT_WP_AND_STATE", 
                               state=smach.CBState(path_follow_cb), 
                               transitions={"continue":"DRIVE_AB","update":"UPDATE_BOOM_STATE","done":"succeeded"}, 
                               remapping={"max_index":"max_index",
                                          "path":"path",
                                          "current_index_in":"current_index",
                                          "current_index_out":"current_index",
                                          "direction_out":"direction",
                                          "current_wp_out":"current_wp",
                                          "boom_state_array":"boom_state_array"
                                          })
        
        smach.StateMachine.add("UPDATE_BOOM_STATE",
                               state=smach_ros.SimpleActionState("/fmImplements/RowClean/move_tool", 
                                                                 move_tool_simpleAction, 
                                                                 goal_slots=['direction','timeout']),
                               transitions={"succeeded":"DRIVE_AB","aborted":"aborted","preempted":"ON_PREEMPTED"},
                               remapping={'direction':'direction','timeout':'timeout'}
                               )
        
        smach.StateMachine.add("DRIVE_AB", 
                               state=smach_ros.SimpleActionState("fmExecutors/follow_path",
                                                                 follow_pathAction,
                                                                 goal_slots=['path']), 
                               transitions = {"succeeded":"SELECT_WP_AND_STATE","aborted":"aborted","preempted":"ON_PREEMPTED"}, 
                               remapping = {"path":"current_wp"})
        
        smach.StateMachine.add("ON_PREEMPTED", 
                               state=smach.CBState(on_preempted), 
                               transitions = {"done":"preempted"}, 
                               remapping={"current_index_in":"current_index","current_index_out":"current_index"})
    
    return nav_sm
    
    
    
    

def build_raise_lower_boom():
    btn_monitors = smach.Concurrence(outcomes=['raise','lower','succeeded'],
                                     default_outcome='succeeded',
                           outcome_map={
                                        "raise":{'MOVE_UP_BTN':'invalid','MOVE_DOWN_BTN':'preempted'}, 
                                        "lower":{'MOVE_UP_BTN':'preempted','MOVE_DOWN_BTN':'invalid'}
                                        },
                           child_termination_cb=force_preempt)
    
    with btn_monitors:
        smach.Concurrence.add("MOVE_UP_BTN", smach_ros.MonitorState("/fmHMI/joy", Joy, up_pressed, -1))
        smach.Concurrence.add("MOVE_DOWN_BTN", smach_ros.MonitorState("/fmHMI/joy", Joy, down_pressed, -1))
        
    sm = smach.StateMachine(outcomes=["succeeded","aborted","preempted"])
    
    with sm:
        smach.StateMachine.add("MONITOR_BUTTONS", 
                               btn_monitors, 
                               transitions={"raise":"MOVE_UP","lower":"MOVE_DOWN","succeeded":"MONITOR_BUTTONS"})
        smach.StateMachine.add("MOVE_UP",
                               smach_ros.SimpleActionState("/fmImplements/RowClean/move_tool",move_tool_simpleAction,goal=move_tool_simpleGoal(direction=1,timeout=10)),
                               transitions = {"succeeded":"MONITOR_BUTTONS","aborted":"aborted","preempted":"preempted"}
                               )
        smach.StateMachine.add("MOVE_DOWN",
                               smach_ros.SimpleActionState("/fmImplements/RowClean/move_tool",move_tool_simpleAction,goal=move_tool_simpleGoal(direction=0,timeout=10)),
                               transitions = {"succeeded":"MONITOR_BUTTONS","aborted":"aborted","preempted":"preempted"}
                               )
    return sm

def build_sm(filename):
    """
        Construct the state machine executing the selected behaviours
    """
    
    
    path,state = load_path(filename)
    
    nav_sm = build_nav_sm(path,state)
    
    master = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    with master:
        smach.StateMachine.add("NAVIGATE_PATH",
                               nav_sm,
                               transitions={"succeeded":"succeeded","aborted":"aborted","preempted":"preempted"})
      
    m2 = behaviours.wii_states.wii_auto_manuel.create(master, "/fmHMI/joy", 2)
    
    m3 = smach.Concurrence(outcomes=['succeeded','aborted','preempted'], 
                           default_outcome='aborted',
                           outcome_map={},
                           child_termination_cb=force_preempt)
    with m3:
        smach.Concurrence.add("MASTER",m2)
        smach.Concurrence.add("MOVE_IMPLEMENT_MANUAL",build_raise_lower_boom())
    
    return m3
    
    
    
if __name__ == "__main__":
    
    rospy.init_node("field_mission")

    filename = rospy.get_param("~filename")
    
    sm = build_sm(filename)
    
    intro_server = smach_ros.IntrospectionServer('field_mission',sm,'/FIELDMISSION')
    intro_server.start()
    
    
    smach_thread = threading.Thread(target = sm.execute)
    smach_thread.start()
    
    rospy.spin();

    sm.request_preempt()
    intro_server.stop()
        
