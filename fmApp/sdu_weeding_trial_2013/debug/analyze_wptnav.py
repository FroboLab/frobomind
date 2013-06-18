#!/usr/bin/env python
#*****************************************************************************
# export
# Copyright (c) 2013, Kjeld Jensen <kjeld@frobomind.org>
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
"""
# imports
import csv
from math import pi
from pylab import *

row1 = [[651224.4126,6133726.5155],[651224.9129,6133727.9296],[651090.0990,6133775.6183],[651089.5987,6133774.2041]]
row2 = [[651223.5332,6133724.0422],[651224.0335,6133725.4563],[651089.2195,6133773.1449],[651088.7193,6133771.7308]]
row3 = [[651222.6538,6133721.5688],[651223.1540,6133722.9830],[651088.3401,6133770.6716],[651087.8399,6133769.2575]]
row4 = [[651221.7744,6133719.0955],[651222.2746,6133720.5097],[651091.2317,6133766.8644],[651090.7315,6133765.4502]]



def pt_in_poly(e,n,p):
	# http://www.ariel.com.au/a/python-point-int-poly.html
    l = len(p)
    inside = False  
    p1e,p1n = p[0]
    for i in range(l+1):
        p2e,p2n = p[i % l]
        if n > min(p1n,p2n):
            if n <= max(p1n,p2n):
                if e <= max(p1e,p2e):
                    if p1n != p2n:
                        einters = (n-p1n)*(p2e-p1e)/(p2n-p1n)+p1e
                    if p1e == p2e or e <= einters:
                        inside = not inside
        p1e,p1n = p2e,p2n
    return inside


file = open ('wptnav_status.txt', 'r')
lines = csv.reader(file, delimiter=',')
dist_ab_l = []
heading_err_l = []
for c1,c2,c3,c4,c5 in lines:
	time = c1
	easting = float (c2)
	northing = float (c3)
	dist_ab = float (c4)
	heading_err = float (c5)
	inside = pt_in_poly (easting, northing, row1)
	if inside:
		if heading_err < 0:
			dist_ab = -dist_ab
		dist_ab_l.append (dist_ab)
		heading_err_l.append (heading_err)
file.close()

print len(dist_ab_l)
plt.figure(1)
title ('Distance from AB line')			
xlabel('Measurement')
ylabel('Distance [m]')
ylim(-0.1,0.1)
grid (True)

plot (dist_ab_l)
plt.figure(2)
title ('Distance from AB line')			
xlabel('Measurement')
ylabel('Distance [m]')
ylim(-0.1,0.1)
grid (True)
plot (heading_err_l)
show()


