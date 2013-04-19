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
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rospy,math

class MarkerUtility():
    """
        Simple utility class to implement different debugging markers
    """
    def __init__(self,topic,frame):
        self.publisher = rospy.Publisher(topic, Marker)
        self.frame = frame
        
    def updateLine(self, point_list):
        marker = Marker()
        marker.points = point_list 
        marker.header.frame_id = self.frame
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.color.a = 0.8
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
       
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0
        marker.pose.position.y = 0 
        marker.pose.position.z = 0
        self.publisher.publish(marker)
    
    def updatePoint(self, point_list):
        marker = Marker()
        marker.points = point_list
        marker.header.frame_id = self.frame
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.a = 0.8
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
       
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0
        marker.pose.position.y = 0 
        marker.pose.position.z = 0
        self.publisher.publish(marker)
        
        
    