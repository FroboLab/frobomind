#!/usr/bin/env python
'''
Created on Feb 7, 2014

@author: leon
'''
import numpy as np

class Controller():
    def __init__(self):
        self.time_step = 0.01
        self.max_time = 10
        
        self.time = np.arange(0,self.max_time,self.time_step)
        self.pos = np.arange(0,self.max_time,self.time_step)
        self.vel = np.arange(0,self.max_time,self.time_step)
        self.acc = np.arange(0,self.max_time,self.time_step)
        
        self.max_jerk = 0.5
        self.max_acc = 2
        self.max_vel = 2
        
        self.old_time = 0
        self.old_pos = 0
        self.old_vel = -1
        self.old_acc = 0
        
        self.user_sp = 2
        self.vel_sp = 0
        self.tolerance = 0.01
        self.zeroband = 1
        self.steps = len(self.time)
    
    def declineAcceleration(self,t):
        self.acc[t] = self.old_acc - self.max_jerk*self.time_step
        if self.acc[t] < -self.max_acc :
            self.acc[t] = -self.max_acc
        elif self.acc[t] > self.max_acc :
            self.acc[t] = self.max_acc
        else:
            if self.acc[t]*self.old_acc < 0 :
                self.acc[t] = self.old_acc
        self.vel[t] = self.old_vel + self.acc[t]*self.time_step
    
    def inclineAcceleration(self,t):
        self.acc[t] = self.old_acc + self.max_jerk*self.time_step
        if self.acc[t] < -self.max_acc :
            self.acc[t] = -self.max_acc
        elif self.acc[t] > self.max_acc :
            self.acc[t] = self.max_acc
        else:
            if self.acc[t]*self.old_acc < 0 :
                self.acc[t] = self.old_acc
        self.vel[t] = self.old_vel + self.acc[t]*self.time_step
        
    def hault(self,t):
        self.acc[t] = 0
        self.vel[t] = self.old_vel            
    
    def enforceMaximum(self,t):                       
        if self.vel[t] > self.max_vel :
            self.vel[t] = self.max_vel
        if self.vel[t] < -self.max_vel :
            self.vel[t] = -self.max_vel 
            
    def upkeep(self,t):
        self.old_pos = self.pos[t]
        self.old_vel = self.vel[t]
        self.old_acc = self.acc[t]

    def isBraking(self):
        return (self.vel_sp == 0) or (self.vel_sp > 0 and self.old_vel < 0) or (self.vel_sp < 0 and self.old_vel > 0)

    def isAtRest(self):
        return self.vel_sp == 0 and (self.old_vel < self.tolerance) and (self.old_vel >- self.tolerance)
        
    def isOnSetpoint(self):
        return (self.old_vel < self.vel_sp + self.tolerance) and (self.old_vel > self.vel_sp - self.tolerance)
        
    def isAccelerating(self):
        return self.vel_sp > self.old_vel

    def isDecelerating(self):
        return self.vel_sp < self.old_vel

    def isDecliningAcceleration(self):
        return self.old_vel > self.vel_sp/2
    
    def isIncliningAcceleration(self):
        return self.old_vel < self.vel_sp/2

    def brake(self,t):
        # brake
        if (self.old_vel > self.zeroband) :
            self.declineAcceleration(t)
        elif self.old_vel < -self.zeroband :
            self.inclineAcceleration(t)
        else :
            if (self.old_vel > self.tolerance) :
                if self.old_acc >= 0 :
                    self.acc[t] = -self.max_acc/2
                    self.vel[t] = self.old_vel + self.acc[t]*self.time_step
                else :
                    self.inclineAcceleration(t)
            elif self.old_vel < -self.tolerance :
                if self.old_acc <= 0 :
                    self.acc[t] = self.max_acc/2
                    self.vel[t] = self.old_vel + self.acc[t]*self.time_step
                else :
                    self.declineAcceleration(t)
            else :
                if (self.user_sp > self.tolerance) :
                    self.inclineAcceleration(t)
                elif self.user_sp < -self.tolerance :
                    self.declineAcceleration(t)
                else :
                    self.acc[t] = 0
                    self.vel[t] = 0

    def simulate(self):
        for t in range(self.steps) : 
            if self.user_sp > self.max_vel :
                self.vel_sp = self.max_vel
            elif self.user_sp < -self.max_vel :
                self.vel_sp = -self.max_vel
            else :
                self.vel_sp = self.user_sp
                
            if self.isBraking() :
                self.vel_sp = 0
                self.brake(t)          
            elif self.isAtRest()  :
                self.hault(t)
            elif self.isOnSetpoint() :
                self.acc[t] = 0
                self.vel[t] = self.old_vel
            elif self.isAccelerating() :
                if self.isDecliningAcceleration() :
                    self.declineAcceleration(t)
                elif self.isIncliningAcceleration() :
                    self.inclineAcceleration(t)
                else:
                    self.acc[t] = self.max_acc
                    self.vel[t] = self.old_vel + self.acc[t]*self.time_step           
            elif self.isDecelerating() :
                if self.isDecliningAcceleration() :
                    self.declineAcceleration(t)
                elif self.isIncliningAcceleration() :  
                    self.inclineAcceleration(t)
                else :
                    self.acc[t] = -self.max_acc
                    self.vel[t] = self.old_vel + self.acc[t]*self.time_step
        
            self.enforceMaximum(t)            
            self.pos[t] = self.old_pos + self.vel[t]*self.time_step
            self.upkeep(t)