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
    This library was developed as an alternative to the far more complex libraries
    available for handling advanced linear algebra and only handles simple 2D vectors.
"""
import numpy as np
import math

class Vector():
    """
        Utility class to handle simple 2D vector calculations
    """
    def __init__(self,a,b):
        """
            Constructor. Implement the concept of a 2D vector
            Usage: vector = Vector(1,2)
        """
        self.vec = [a,b]
    
    def __sub__(self,other):
        """
            Method to handle subtraction of two vectors.
            Usage: result = vector_a - vector_b
        """        
        return Vector(self.vec[0] - other[0] , self.vec[1] - other[1])
    
    def __neg__(self):
        """
            Method to handle negation of a vector vectors.
            Usage: result = -vector_a
        """        
        return Vector(-self[0],-self[1])
    
    def __add__(self,other):
        """
            Method to handle addition of two vectors
            Usage: result = vector_a + vector_b
        """  
        return Vector(self.vec[0] + other[0] , self.vec[1] + other[1])
    
    def __getitem__(self,k):
        """
            Accessor. Access vector as list or tuple
            Usage: x_component = vector[0] 
        """  
        return self.vec[k]
    
    def __setitem__(self,k,value):
        """
            Mutator. Mutate vector as list or tuple
            Usage: vector[1] = y_component
        """        
        self.vec[k] = value
        
    def length(self):
        """
            Method returning the length of a vector
            Usage: dist = vector.length()
        """    
        return math.sqrt(np.dot(self.vec,self.vec))
    
    def dotWith(self,other):
        """
            Method returning the dot product of two vectors
            Usage: dot = vector_a.dotWith(vector_b)
        """ 
        return np.dot(self.vec,other.vec)
    
    def angle(self,other):
        """
            Method returning angle between two vectors. Beware the sine trap!
            Usage: angle = vector_a.angle(vector_b)
        """ 
        if self.length() and other.length() :
            tmp = np.dot(self.vec,other.vec) / (self.length() * other.length())
            if tmp > 1 :
                return math.acos(1)
            elif  tmp < -1 :
                return math.acos(-1)
            else :
                return math.acos(tmp)
        else:
            return 0.0
    
    def hat(self):
        """
            Method returning the perpendicular vector
            Usage: hat_vector = vector_a.hat()
        """ 
        return Vector(self.vec[1],-self.vec[0])
    
    def rotate(self,rad):
        """
            Method returning a vector rotated according to input
            Usage: new_vector = vector_a.rotate(math.pi/3)
        """ 
        new_x = math.cos(rad)*self.vec[0] - math.sin(rad)*self.vec[1]
        new_y = math.sin(rad)*self.vec[0] + math.cos(rad)*self.vec[1]
        return Vector(new_x,new_y)
    
    def projectedOn(self,other):
        """
            Method returning a projection of the vector
            Usage: projection = vector_a.projectedOn(vector_b)
        """ 
        if other.length() :
            tmp = np.dot(other.vec,self.vec) / (other.length() * other.length())
            new = Vector(other.vec[0],other.vec[1])
            return new.scale(tmp)
        else :
            print("Vector was projected on the zero vector")
            return Vector(0,0)
    
    def scale(self,num):
        """
            Method returning the vector scaled by a scalar
            Usage: new_vector = vector_a.scale(7.3)
        """ 
        return Vector(self.vec[0] * num , self.vec[1] * num)
    
    def unit(self):
        """
            Method returning the vector normalised too length 1
            Usage: norm_vector = vector_a.unit()
        """ 
        if self.length() :
            return Vector(self.vec[0] / self.length() , self.vec[1] / self.length())
        else :
            return Vector(0,0)
    