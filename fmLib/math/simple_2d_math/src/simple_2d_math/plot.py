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
from itertools import chain
import matplotlib.pyplot as pyplot
import copy

class Vectorplot():
    """
        Utility class to handle simple plots in 2D.
        The class holds lists of points, vectors and lines
        When show() is called the elements are plotted.
    """
    def __init__(self,x_min,x_max,y_min,y_max):
        self.vectors = []
        self.points = []
        self.lines_x = []
        self.lines_y = []
        self.poses_x = []
        self.poses_y = []        
        self.plotrange = [x_min,x_max,y_min,y_max]
        self.plot = pyplot.figure()
        
    def show(self):
        pyplot.figure()
        if len(self.vectors) > 0 :
            pyplot.quiver(*zip(*map(lambda l: chain(*l), self.vectors)), 
                      angles='xy', scale_units='xy', scale=1)
        if len(self.points) > 0 :
            x,y = zip(*map(lambda l: l,self.points))
            pyplot.plot(list(x),list(y), 'ro')
        if len(self.lines_x) > 0 :
            pyplot.plot(self.lines_x,self.lines_y, '-')
        if len(self.poses_x) > 0 :
            pyplot.plot(self.poses_x,self.poses_y, color='red')
        pyplot.axis('equal')
        pyplot.axis(self.plotrange)
        pyplot.grid()
        pyplot.show()
        
    def draw(self):
        self.plot.draw()
        
    def addVector(self,origin,direction):
        self.vectors.append(copy.deepcopy( (origin,direction) ))
        
    def addPoint(self,tup):
        self.points.append(tup)
        
    def addLine(self,tup_from,tup_to):
        self.lines_x.append(tup_from[0])
        self.lines_y.append(tup_from[1])
        self.lines_x.append(tup_to[0])
        self.lines_y.append(tup_to[1])
        
    def addPose(self,tup):
        self.poses_x.append(tup[0])
        self.poses_y.append(tup[1])