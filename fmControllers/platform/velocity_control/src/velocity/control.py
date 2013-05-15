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
import rospy,math
from geometry_msgs.msg import TwistStamped
from simple_2d_math.vector import Vector

class Controller():
    def __init__(self):      
        # Init control loop
        self.twist = TwistStamped()
        self.lin_err = 0.0
        self.ang_err = 0.0
        self.lin_prev_err = 0.0
        self.ang_prev_err = 0.0
        self.lin_int = 0.0
        self.ang_int = 0.0
        self.lin_diff = 0.0
        self.ang_diff = 0.0
        self.fb_linear = 0.0
        self.fb_angular = 0.0   
        
        # Get parameters
        self.period = rospy.get_param("~period",0.1)
        self.lin_p = rospy.get_param("~lin_p",0.4)
        self.lin_i = rospy.get_param("~lin_i",0.6)
        self.lin_d = rospy.get_param("~lin_d",0.0)
        self.ang_p = rospy.get_param("~ang_p",0.8)
        self.ang_i = rospy.get_param("~ang_i",0.1)
        self.ang_d = rospy.get_param("~ang_d",0.05)
        self.int_max = rospy.get_param("~integrator_max",0.1)   
        self.filter_size = rospy.get_param("~filter_size",10) 
        self.max_linear_velocity = rospy.get_param("~max_linear_velocity",2)
        self.max_angular_velocity = rospy.get_param("~max_angular_velocity",1)
        self.use_dynamic_reconfigure = rospy.get_param("~use_dynamic_reconfigure",False)
        self.linear_vel = [0.0] * self.filter_size
        self.angular_vel = [0.0] * self.filter_size
        self.time = 0.0
        self.last_entry = rospy.Time.now()
        self.last_cl_entry = rospy.Time.now()
        self.distance = 0.0
        self.last_position = Vector(1,0)
        self.angle = 0.0
        self.last_heading = Vector(1,0)
        self.ptr = 0
        
    def reconfigure_cb(self, config, level):
        self.lin_p = config['lin_p']
        self.lin_i = config['lin_i']
        self.lin_d = config['lin_d']
        self.ang_p = config['ang_p']
        self.ang_i = config['ang_i']
        self.ang_d = config['ang_d']
        self.int_max = config['integrator_max']   
        return config
    
    def generateTwist(self,sp_linear,sp_angular):
        # Calculate time since last entry
        self.period = (rospy.Time.now() - self.last_cl_entry).to_sec()
        self.last_cl_entry = rospy.Time.now()
        
        # Estimate feedback velocities
        self.fb_linear = (sum(self.linear_vel)/len(self.linear_vel))
        self.fb_angular = -(sum(self.angular_vel)/len(self.angular_vel))
        
        # Calculate velocity errors for control loop
        self.lin_err = sp_linear - self.fb_linear
        self.ang_err = sp_angular - self.fb_angular
        
        # Calculate integrators and implement max
        self.lin_int += self.lin_err * self.period
        if self.lin_int > self.int_max :
            self.lin_int = self.int_max
        if self.lin_int < -self.int_max :
            self.lin_int = -self.int_max                
        self.ang_int += self.ang_err * self.period    
        if self.ang_int > self.int_max :
            self.ang_int = self.int_max
        if self.ang_int < -self.int_max :
            self.ang_int = -self.int_max
            
        # Calculate differentiators and save value
        if self.period :
            self.lin_diff = (self.lin_prev_err - self.lin_err) / self.period
            self.ang_diff = (self.ang_prev_err - self.ang_err) / self.period
        self.lin_prev_err = self.lin_err
        self.ang_prev_err = self.ang_err
        
        # Update twist message with control velocities
        self.twist.header.stamp = rospy.Time.now() 
        self.twist.twist.linear.x = sp_linear + (self.lin_err * self.lin_p) + (self.lin_int * self.lin_i) + (self.lin_diff * self.lin_d)
        self.twist.twist.angular.z = sp_angular + (self.ang_err * self.ang_p) + (self.ang_int * self.ang_i) + (self.ang_diff * self.ang_d)
#        print("Fb:",(self.fb_linear,self.fb_angular)," Sp:",(sp_linear,sp_angular)," New:",(self.twist.twist.linear.x,self.twist.twist.angular.z))
        
        # Implement maximum linear velocity and maximum angular velocity
        if self.twist.twist.linear.x > self.max_linear_velocity:
            self.twist.twist.linear.x = self.max_linear_velocity
        if self.twist.twist.linear.x < -self.max_linear_velocity:
            self.twist.twist.linear.x = -self.max_linear_velocity
        if self.twist.twist.angular.z > self.max_angular_velocity:
            self.twist.twist.angular.z = self.max_angular_velocity
        if self.twist.twist.angular.z < -self.max_angular_velocity:
            self.twist.twist.angular.z = -self.max_angular_velocity
            
        return self.twist

    def setFeedback(self,position,heading):
        # Calculate time since last entry
        self.time = (rospy.Time.now() - self.last_entry).to_sec()
        self.last_entry = rospy.Time.now()
        
        # Calculate distance travelled since last entry
        self.distance = (self.last_position - position).length()
        self.last_position = position
        
        # Calculate change in orientation since last entry
        self.angle = heading.angle(self.last_heading)
        t1 = heading.rotate(self.angle)
        if self.last_heading.angle(t1) != 0 :
            self.angle = -self.angle
            
        self.last_heading = heading
        
        if self.time :
            self.linear_vel[self.ptr] = self.distance / self.time
            self.angular_vel[self.ptr] = self.angle / self.time
            self.ptr = self.ptr + 1
            if self.ptr >= self.filter_size :
                self.ptr = 0  
