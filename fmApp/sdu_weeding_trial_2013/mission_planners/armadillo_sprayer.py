#!/usr/bin/env python
import roslib; 
roslib.load_manifest("fmDecisionMakers")
import rospy

import smach
import smach_ros

import threading

import behaviours
import behaviours.wii_states.wii_auto_manuel
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from fmExecutors.msg import sprayAction,sprayGoal

import numpy
import math

def do_abort(arg):
    return True

def displace_AB_line(path,offset):
    """
        Displaces a AB line by creating a line local coordinate system where 
        x is pointing in the direction of the line, the offset is then the y part
        of this local coordinate system
    """
    
    # convert positions to matrix
    a = numpy.transpose(numpy.matrix([path.poses[0].pose.position.x,path.poses[0].pose.position.y],float))
    b = numpy.transpose(numpy.matrix([path.poses[1].pose.position.x,path.poses[1].pose.position.y],float))
    
    # calculate line vector
    v = b-a
    # calculate the angle of the line local coordinate system
    an = math.atan2(v[1],v[0])
    
    # create rotation matrix for which to rotate the local displacement into the coordinate system
    rot = numpy.matrix([[math.cos(an),-math.sin(an)],[math.sin(an),math.cos(an)]],float)
    # create a point in the line local coordinate system which will be rotated and translated
    # into new_a and new_b
    disp = numpy.transpose(numpy.matrix([0,offset],float))
    
    # rotate the point
    xy_rot = rot*disp
    
    # translate the point with respect to a and b
    a_trans = xy_rot + a
    b_trans = xy_rot + b
    
    # update the path with the offsetted line
    path.poses[0].pose.position.x = a_trans[0]
    path.poses[0].pose.position.y = a_trans[1]
    
    path.poses[1].pose.position.x = b_trans[0]
    path.poses[1].pose.position.y = b_trans[1]
    

def load_path(filename):
    path = Path()
    
    with open(filename) as file:
        for line in file:
            if not line.startswith("#"):
                p = PoseStamped()
                x,y=line.split()
                p.header.frame_id = "odom_combined"
                p.pose.position.x = float(x)
                p.pose.position.y = float(y)
                path.poses.append(p)
    
    return path

def load_path2(filename):
    path = Path()
    first = True
    off_x = 0
    off_y = 0
    with open(filename) as file:
        for line in file:
            if not line.startswith("#"):
                p = PoseStamped()
                y,x=line.split()
                p.header.frame_id = "odom_combined"
                if first:
                    p.pose.position.x = 0.0
                    p.pose.position.y = 0.0
                    off_x = float(x)
                    off_y = float(y)
                    first = False 
                else:
                    p.pose.position.x = float(x) - off_x
                    p.pose.position.y = float(y) - off_y
                path.poses.append(p)
    
    return path

def build_nav_sm(fn,offset,rev,spray):
    
    path = load_path(fn)
    
    if rev:
        path.poses.reverse()
    
    #displace_AB_line(path, offset)
    
    pf = behaviours.PlanFollow(path, "/fmExecutors/follow_path")
    
    plan_sm = smach.StateMachine(outcomes=["succeeded","aborted","preempted"])
    
    with plan_sm:
        smach.StateMachine.add("FOLLOW_PLAN",
                               pf,
                               transitions={"succeeded":"DONE","aborted":"aborted","preempted":"preempted"})
        smach.StateMachine.add("DONE",
                               behaviours.wait_state.WaitState(rospy.Duration(1)),
                               transitions={"succeeded":"DONE"})
        
    if spray:
        auto_sm = smach.Concurrence(outcomes=["succeeded","aborted","preempted"], default_outcome="succeeded", child_termination_cb=do_abort)
        
        with auto_sm:
            smach.Concurrence.add("FOLLOW_PLAN",
                                  plan_sm)
            smach.Concurrence.add("SPRAY",
                                  smach_ros.SimpleActionState("/spray", sprayAction, goal=sprayGoal(distance=0.25))
                                  )
    else:
        auto_sm = plan_sm
    
    sm = behaviours.wii_states.wii_auto_manuel.create(auto_sm, "/fmHMI/joy", 2)
    
    return sm


if __name__ == "__main__":
    rospy.init_node("field_mission")
    
    fn = rospy.get_param("~path_file")
    offset = rospy.get_param("~offset")
    rev = rospy.get_param("~reverse")
    spray = rospy.get_param("~spray")
    
    master = build_nav_sm(fn,offset,rev,spray)
    
    intro_server = smach_ros.IntrospectionServer('field_mission',master,'/FIELDMISSION')
    intro_server.start()    
    
    smach_thread = threading.Thread(target = master.execute)
    smach_thread.start()
    
    rospy.spin();

    master.request_preempt()
    intro_server.stop()
    
