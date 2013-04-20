#!/usr/bin/env python
#*****************************************************************************
# KP2000 projection conversion test
# Copyright (c) 2013, Leon Bonde Larsen <leon@frobomind.org>
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
#*****************************************************************************
"""
    This script was developed prior to the position action server and line planner.
    It implements the basic ideas of the planners and were used both for debugging
    and to test stability in odd situations
"""
import math
from simple_2d_math.plot import Vectorplot
from simple_2d_math.vector import Vector
   
class Planner():
    def __init__(self):
        # Task parameters
        self.line_begin = Vector(-2,-4)
        self.line_end = Vector(2,5)
        self.line = Vector(self.line_end[0]-self.line_begin[0],self.line_end[1]-self.line_begin[1])
        
        
        self.position = Vector(0,-6)
        self.heading = math.pi/6
        

        # Robot parameters
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.max_linear_velocity = 2 #m/s
        self.max_angular_velocity = 2 #rad/s
        
        # General planner parameters
        self.target_radius = 0.1
        self.max_angle_error = 0.1
        self.retarder = 0.1
        
        # Line planner parameters
        self.rabbit_factor = 0.2
        
        # Position planner parameters
        self.linear_scale_factor = 1
        self.angular_scale_factor = 2
   
        # Simulation parameters
        self.max_steps = 1000
        self.stepsize = 0.1 # sec/step
        
        # Init simulation
        self.steps = 0
        self.lin_vel = []
        self.ang_vel = []       
        
        # init plot
        self.plot = Vectorplot(-10, 10, -10, 10)
#        self.plot.addPoint(self.line_begin)
#        self.plot.addPoint(self.line_end)
#        self.plot.addLine(self.line_begin,self.line_end)
        
        # init position planner
        self.destination = Vector(self.line_end[0],self.line_end[1])
        path = self.destination - Vector(self.position[0], self.position[1])    
        self.distance_error = path.length()
        self.angle_error = 0.0

    def updatePos(self):
        self.heading = self.heading + (self.angular_velocity*self.stepsize);
        self.position[0] = self.position[0] + (self.linear_velocity*self.stepsize) * math.cos(self.heading)
        self.position[1] = self.position[1] + (self.linear_velocity*self.stepsize) * math.sin(self.heading)
        
    def step(self):
#        self.positionPlanner(self.line_end)
        self.linePlanner()
        self.steps = self.steps + 1
        self.lin_vel.append(math.fabs(self.linear_velocity))
        self.ang_vel.append(math.fabs(self.angular_velocity))
#        if not self.steps%20 :
#            self.plot.addVector(self.position,Vector(math.cos(self.heading),math.sin(self.heading)))
        
    def targetReached(self):
        return math.sqrt(math.pow(self.line_end[0]-self.position[0],2) + math.pow(self.line_end[1]-self.position[1],2)) < self.target_radius
    
    def spin(self):
        self.plot.addVector(self.position,Vector(math.cos(self.heading),math.sin(self.heading)))
        while not self.targetReached() and self.steps < self.max_steps:
            self.plot.addPose(self.position)
            self.step()
            self.updatePos()
        self.plot.addPose(self.position)

    def show(self):
#        print(sum(self.lin_vel)/len(self.lin_vel))
#        print(sum(self.ang_vel)/len(self.ang_vel))
        self.plot.addVector(self.position,Vector(math.cos(self.heading),math.sin(self.heading)))
        self.plot.addPoint(self.position)
        self.plot.show()
    
    def positionPlanner(self,goal):
        self.destination[0] = goal[0]
        self.destination[1] = goal[1]
        
        # Construct desired path vector and calculate distance error
        path = self.destination - Vector(self.position[0], self.position[1])
        
        # Calculate distance error     
        self.distance_error = path.length()

        # Construct heading vector
        head = Vector(math.cos(self.heading), math.sin(self.heading))
                
        # Calculate angle between heading vector and path vector
        self.angle_error = head.angle(path)

        # Rotate the heading vector according to the calculated angle and test correspondence
        # with the path vector. If not zero sign must be flipped. This is to avoid the sine trap.
        t1 = head.rotate(self.angle_error)
        if path.angle(t1) != 0 :
            self.angle_error = -self.angle_error          
        
        # Generate twist from distance and angle errors (For now simply 1:1)
        self.linear_velocity = self.distance_error * self.linear_scale_factor
        self.angular_velocity = self.angle_error * self.angular_scale_factor
        
        if math.fabs(self.angle_error) > self.max_angle_error :
            self.linear_velocity *= self.retarder
            
        # Implement maximum linear_velocity velocity and maximum angular_velocity velocity
        if self.linear_velocity > self.max_linear_velocity:
            self.linear_velocity = self.max_linear_velocity
        if self.linear_velocity < -self.max_linear_velocity:
            self.linear_velocity = -self.max_linear_velocity
        if self.angular_velocity > self.max_angular_velocity:
            self.angular_velocity = self.max_angular_velocity
        if self.angular_velocity < -self.max_angular_velocity:
            self.angular_velocity = -self.max_angular_velocity
        
    def linePlanner(self):
        # Project position vector on line vector
        proj = (self.position - self.line_begin).projectedOn(self.line)
        
        perp = proj - self.position
        if perp.length() :
            self.rabbit_factor = 1/perp.length()
           
        # Construct rabbit point
        rabbit = self.line - proj
        rabbit = rabbit.scale(self.rabbit_factor)
        rabbit += self.line_begin
        rabbit += proj
           
        # Call positionPlanner
#        self.plot.addPoint(rabbit)
        self.positionPlanner(rabbit)
    
    def test(self):
        self.position = Vector(-8,4)
        self.heading = Vector(0,1)
        self.plot.addPoint(self.position)
        
        for i in range(8) :
            (next,next_next) = self.addRightTurn(self.position,self.heading)
            self.plot.addLine(self.position,next)
            self.plot.addLine(next,next_next)
            self.plot.addPoint(next)
            self.plot.addPoint(next_next)
            
            self.heading = -self.heading
            self.position = next_next
            
            (next,next_next) = self.addLeftTurn(self.position,self.heading)
            self.plot.addLine(self.position,next)
            self.plot.addLine(next,next_next)
            self.plot.addPoint(next)
            self.plot.addPoint(next_next)
            
            self.heading = -self.heading
            self.position = next_next
      
    
#        self.plot.addVector(next_point,next_heading)
    
        
        self.plot.show()
        
    def addRightTurn(self,position,heading):
        next_point = position + heading.hat().unit().scale(1)
        next_heading = -heading
        next_next_point = next_point + next_heading.unit().scale(8)
        return(next_point,next_next_point)
    
    def addLeftTurn(self,position,heading):
        next_point = position - heading.hat().unit().scale(1)
        next_heading = -heading
        next_next_point = next_point + next_heading.unit().scale(8)
        return(next_point,next_next_point)
        
            
if __name__ == '__main__':
    planner = Planner()
    planner.test()
    #planner.spin()
    #planner.show()

#class CasmoPlanner():
#    def __init__(self):
#        self.odom_frame = rospy.get_param("~odom_frame","/odom")
#        
#        self.position = Vector(0,0)
#        self.heading = Vector(0,0)
#        self.next_turn = 'none'
#        
#        self.first_endpoint_undefined = True
#        self.second_endpoint_undefined = True
#        self.first_endpoint = Vector(0,0)
#        self.second_endpoint = Vector(0,0)
#    
#    def getTransform(self):     
#        try:
#            (self.position,head) = self.__listen.lookupTransform( self.odom_frame,self.base_frame,rospy.Time(0)) # The transform is returned as position (x,y,z) and an orientation quaternion (x,y,z,w).
#            (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(head)
#            self.heading = Vector(math.cos(self.yaw), math.sin(yaw))            
#        except (tf.LookupException, tf.ConnectivityException),err:
#            rospy.loginfo("could not locate vehicle")
#            
#    def update(self,point_list,direction):
#        self.getTransform()
#        if self.first_endpoint_undefined :
#            self.first_endpoint = self.position.projectedOn(self.heading).unit().scale(100.0)
#            point_list.append( self.first_endpoint )
#            self.first_endpoint_undefined = False
#            if 'left' in direction :
#                self.insertLeftTurn()
#                self.next_turn = 'right'
#            elif 'right' in direction :
#                self.insertLeftTurn()
#                self.next_turn = 'right'
#            else:
#                print("Casmo update called without indication of side")
#        elif self.second_endpoint_undefined :
#            self.second_endpoint = 
#            self.insertRightTurn()
#    
#    def leftPerpendicular(self,vector):
#        v1 = vector.hat()
