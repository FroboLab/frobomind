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
# Change log:
# 26-Mar 2013 Leon: Changed to using tf for quaternion calculations
#                   Turned coordinate system to match odom frame
#****************************************************************************/
import rospy, tf, actionlib, math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from position_action_server.msg import positionAction
from tf import TransformListener

class vector():
    """
        Utility class to handle simple 2D vector calculations
    """
    def __init__(self,a,b):
        self.vec = [a,b]
    
    def __sub__(self,other):
        return vector(self.vec[0] - other[0] , self.vec[1] - other[1])
    
    def __getitem__(self,k):
        return self.vec[k]
        
    def length(self):
        return math.sqrt(np.dot(self.vec,self.vec))
    
    def angle(self,other):
        tmp = np.dot(self.vec,other.vec) / (self.length() * other.length())
        if tmp > 1 :
            return math.acos(1)
        elif  tmp < -1 :
            return math.acos(-1)
        else :
            return math.acos(tmp)
    
    def hat(self):
        return vector(self.vec[1],-self.vec[0])
    
    def rotate(self,rad):
        new_x = math.cos(rad)*self.vec[0] - math.sin(rad)*self.vec[1]
        new_y = math.sin(rad)*self.vec[0] + math.cos(rad)*self.vec[1]
        return vector(new_x,new_y)

class positionGoalActionServer():
    """
        Action server taking position goals and generating twist messages accordingly
    """
    def __init__(self,name):
        # Get topics and transforms from parameter server
        self.max_linear_velocity = rospy.get_param("~max_linear_velocity",2)
        self.max_angular_velocity = rospy.get_param("~max_angular_velocity",1)
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic","/fmSignals/cmd_vel")
        self.odom_frame = rospy.get_param("~odom_frame","/odom")
        self.base_frame = rospy.get_param("~base_frame","/base_footprint")
        
#        # Reduce max speed
#        self.max_linear_velocity /= 2
#        self.max_angular_velocity /= 2
        
        # Get parameters
        self.max_distance_error = rospy.get_param("~max_distance_error",0.2)
        self.straight_line = rospy.get_param("~straight_line",False)
        self.use_tf = rospy.get_param("~use_tf",False)        

        # Setup Publishers and subscribers
        if not self.use_tf :
            self.odometry_topic = rospy.get_param("~odometry_topic","/fmKnowledge/odom")
            self.odom_sub = rospy.Subscriber(self.odometry_topic, Odometry, self.onOdometry )
        self.twist_pub = rospy.Publisher(self.cmd_vel_topic, TwistStamped)
                
        # Parameters for action server
        self.period = 0.1
        self.retarder_point = 0.3 #distance to target when speed should start declining
        
        # Parameters for control loop
        self.lin_p = 0.4
        self.lin_i = 0.6
        self.lin_d = 0.0
        self.ang_p = 0.8
        self.ang_i = 0.1
        self.ang_d = 0.05
        self.int_max = 0.1
        self.retarder = 0.8
        self.max_angle_error = math.pi/4

        self.max_linear_velocity = 0.4
        self.max_angular_velocity = 1.8
        # Init control loop
        self.lin_err = 0.0
        self.ang_err = 0.0
        self.lin_prev_err = 0.0
        self.ang_prev_err = 0.0
        self.lin_int = 0.0
        self.ang_int = 0.0
        self.lin_diff = 0.0
        self.ang_diff = 0.0
        self.distance_error = 0
        self.angle_error = 0
        self.fb_linear = 0.0
        self.fb_angular = 0.0
        self.sp_linear = 0.0
        self.sp_angular = 0.0
        
        # Init action server
        self.rate = rospy.Rate(1/self.period)
        self.twist = TwistStamped()
        self.destination = vector(0,0)
        self.position = vector(0,0)
        self.quaternion = np.empty((4, ), dtype=np.float64)
        
        # Init TF listener
        self.__listen = TransformListener()
        
        # Init action server      
        self._action_name = name
        self._server = actionlib.SimpleActionServer(self._action_name, positionAction, auto_start=False, execute_cb=self.execute)
        self._server.register_preempt_callback(self.preempt_cb);
        self._server.start()
        
    def preempt_cb(self):                   
        # Publish zero twist message
        self.twist.header.stamp = rospy.Time.now()
        self.twist.twist.linear.x = 0
        self.twist.twist.angular.z = 0            
        self.twist_pub.publish(self.twist)
        #self._server.set_preempted()
   
    def get_current_position(self):
        if self.use_tf :
            ret = False
            try:
                (self.position,self.heading) = self.__listen.lookupTransform( self.odom_frame,self.base_frame,rospy.Time(0)) # The transform is returned as position (x,y,z) and an orientation quaternion (x,y,z,w).
                ret = True
            except (tf.LookupException, tf.ConnectivityException),err:
                rospy.loginfo("could not locate vehicle")
            return ret
            

    def execute(self,goal):
        # Construct a vector from position goal
        self.destination.vec[0] = goal.x
        self.destination.vec[1] = goal.y  
        rospy.loginfo(rospy.get_name() + "Received goal: (%f,%f) ",goal.x,goal.y)
        
        while not rospy.is_shutdown() :
            # Check for new goal
            if self._server.is_new_goal_available() :
                break

            # Preemption check
            if self._server.is_preempt_requested():
                break
            
            # Get current position
            self.get_current_position()

            # Construct desired path vector and calculate distance error
            path = self.destination - vector(self.position[0], self.position[1])
            self.distance_error = path.length()
                  
            # If position is unreached
            if self.distance_error > self.max_distance_error :
                
                # Calculate yaw and construct a heading vector
                if self.use_tf :
                    (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(self.heading)
                else :
                    (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(self.quaternion)

                # Construct heading vector
                head = vector(math.cos(yaw), math.sin(yaw))
                
                # Calculate angle between heading vector and path vector
                self.angle_error = head.angle(path)

                # Rotate the heading vector according to the calculated angle and test correspondence
                # with the path vector. If not zero sign must be flipped. This is to avoid the sine trap.
                t1 = head.rotate(self.angle_error)
                if path.angle(t1) != 0 :
                    self.angle_error = -self.angle_error
                 
                # Generate setpoints from distance and angle errors (For now simply 1:1)
                self.sp_linear = self.distance_error 
                self.sp_angular = self.angle_error  
                
                # Calculate control error
                self.lin_err = self.sp_linear - self.fb_linear
                self.ang_err = self.sp_angular - self.fb_angular
                
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
                self.lin_diff = (self.lin_prev_err - self.lin_err) / self.period
                self.ang_diff = (self.ang_prev_err - self.ang_err) / self.period
                self.lin_prev_err = self.lin_err
                self.ang_prev_err = self.ang_err
                
                # Update twist
                self.twist.twist.linear.x = (self.lin_err * self.lin_p) + (self.lin_int * self.lin_i) + (self.lin_diff * self.lin_d)
                self.twist.twist.angular.z = (self.ang_err * self.ang_p) + (self.ang_int * self.ang_i) + (self.ang_diff * self.ang_d)
                
                # Implement retarders
                if self.distance_error < self.retarder_point :
                    self.twist.twist.linear.x *=  self.retarder
        
                if math.fabs(self.angle_error) > self.max_angle_error :
                    self.twist.twist.linear.x *= self.retarder
               
                # Implement maximum linear velocity and maximum angular velocity
                if self.twist.twist.linear.x > self.max_linear_velocity:
                    self.twist.twist.linear.x = self.max_linear_velocity
                if self.twist.twist.linear.x < -self.max_linear_velocity:
                    self.twist.twist.linear.x = -self.max_linear_velocity
                if self.twist.twist.angular.z > self.max_angular_velocity:
                    self.twist.twist.angular.z = self.max_angular_velocity
                if self.twist.twist.angular.z < -self.max_angular_velocity:
                    self.twist.twist.angular.z = -self.max_angular_velocity
                
                print("dist_err:",self.distance_error," ang_err:",self.angle_error)
                print("sp_lin:",self.sp_linear," sp_ang:",self.sp_angular) 
                print("fb_lin:",self.fb_linear," b_ang:",self.fb_angular) 
                print("cmd_lin:",self.twist.twist.linear.x," cmd_ang:",self.twist.twist.angular.z)
                print("")
                
                # If not preempted, add a time stamp and publish the twist
                if not self._server.is_preempt_requested() :     
                    self.twist.header.stamp = rospy.Time.now()               
                    self.twist_pub.publish(self.twist)
                
                # Block   
                try :
                    self.rate.sleep()
                except rospy.ROSInterruptException:
                    return 'preempted'
            else:
                # Succeed the action - position has been reached
                self._server.set_succeeded()
                
                # Publish a zero twist to stop the robot
                self.twist.header.stamp = rospy.Time.now()
                self.twist.twist.linear.x = 0
                self.twist.twist.angular.z = 0
                self.twist_pub.publish(self.twist)                
                break
        # Return statement
        if self._server.is_preempt_requested() :
            self._server.set_preempted()
            return 'preempted' 
        elif rospy.is_shutdown() :
            return 'aborted'
        else :               
            return 'succeeded'
        
    def onOdometry(self, msg):
        # Extract the orientation quaternion
        self.quaternion[0] = msg.pose.pose.orientation.x
        self.quaternion[1] = msg.pose.pose.orientation.y
        self.quaternion[2] = msg.pose.pose.orientation.z
        self.quaternion[3] = msg.pose.pose.orientation.w
        
        # Extract the position vector
        self.position.vec[0] = msg.pose.pose.position.x
        self.position.vec[1] = msg.pose.pose.position.y
        
        # Extract twist
        self.fb_linear = msg.twist.twist.linear.x
        self.fb_angular = msg.twist.twist.angular.z       
        
if __name__ == '__main__':
    try:
        rospy.init_node('positionAction')
        action_server = positionGoalActionServer(rospy.get_name())
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass        
