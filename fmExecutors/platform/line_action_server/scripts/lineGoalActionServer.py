#!/usr/bin/env python
#/****************************************************************************
# FroboMind positionGoalActionServer.py
# Copyright (c) 2011-2013, author Leon Bonde Larsen <leon@bondelarsen.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name FroboMind nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/
"""
    Action server interface for line planner.
    Takes a line goal defining begin point and end point
"""
import rospy,actionlib
from line_action_server.msg import lineAction
from line_control.planner import LinePlanner

class LineGoalActionServer():
    def __init__(self,name):
        # Init action server      
        self._action_name = name
        self._server = actionlib.SimpleActionServer(self._action_name, lineAction, auto_start=False, execute_cb=self.execute)
        self._server.register_preempt_callback(self.preempt_cb);
        
        # Init planner
        self._planner = LinePlanner()
        self._planner.isNewGoalAvailable = self._server.is_new_goal_available
        self._planner.isPreemptRequested = self._server.is_preempt_requested
        self._planner.setSucceeded = self._server.set_succeeded
        self._planner.setPreempted = self._server.set_preempted
        
        # Start the action server
        self._server.start()
    
    def preempt_cb(self):                   
        self._planner.stop()
    
    def execute(self,goal):
        self._planner.execute(goal)    
        
if __name__ == '__main__':
    try:
        rospy.init_node('lineAction')
        action_server = LineGoalActionServer(rospy.get_name())
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass        
